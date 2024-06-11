import math
import os
import numpy as np
import rospy
import threading
from geometry_msgs.msg import Twist
from code_llm.msg import Observations

robot_nodes = {}
thread_local = threading.local()


class RobotNode:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.ros_initialized = False
        self.velocity_publisher = None
        self.robot_info = {
            "position": np.array([0.0, 0.0]),
            "radius": 0.0,
            "velocity": np.array([0.0, 0.0]),
        }
        self.timer = None
        self.init_position = None
        self.obstacles_info = []
        self.other_robots_info = []
        self.prey_position = np.array([0.0, 0.0])

    def observation_callback(self, msg: Observations):
        self.obstacles_info = []
        self.other_robots_info = []
        for obj in msg.observations:
            if obj.type == "Robot":
                if obj.id == self.robot_id:
                    self.robot_info["position"] = np.array([obj.position.x, obj.position.y])
                    if self.init_position is None:
                        self.init_position = self.robot_info["position"]
                    self.robot_info["radius"] = obj.radius
                    continue
                self.other_robots_info.append(
                    {
                        "id": obj.id,
                        "position": np.array([obj.position.x, obj.position.y]),
                        "velocity": np.array([obj.velocity.linear.x, obj.velocity.linear.y]),
                        "radius": obj.radius,
                    }
                )
            elif obj.type == "Obstacle":
                self.obstacles_info.append(
                    {
                        "id": obj.id,
                        "position": np.array([obj.position.x, obj.position.y]),
                        "radius": obj.radius,
                    }
                )

    def initialize_ros_node(self):
        if not self.ros_initialized:
            self.ros_initialized = True
            rospy.Subscriber(f"/observation", Observations, self.observation_callback)
            self.velocity_publisher = rospy.Publisher(f"/robot_{self.robot_id}/velocity", Twist, queue_size=10)

            current_folder = os.path.dirname(os.path.abspath(__file__))
            rospy.set_param("data_path", str(current_folder) + "/data")

            print(f"Waiting for position message from /robot_{self.robot_id}/observation...")
            msg = rospy.wait_for_message(f"/observation", Observations)
            self.observation_callback(msg)
            print(f"Observations data init successfully")
            self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_velocities)

    def publish_velocities(self,event):
        velocity_msg = Twist()
        velocity_msg.linear.x = self.robot_info["velocity"][0]
        velocity_msg.linear.y = self.robot_info["velocity"][1]
        self.velocity_publisher.publish(velocity_msg)

    def get_position(self):
        return self.robot_info["position"]

    def get_velocity(self):
        return self.robot_info["velocity"]

    def get_radius(self):
        return self.robot_info["radius"]

    def set_velocity(self, velocity):
        self.robot_info["velocity"] = np.array(velocity)

    def get_surrounding_robots_info(self):
        return self.other_robots_info

    def get_surrounding_obstacles_info(self):
        return self.obstacles_info

    def get_prey_position(self):
        return self.prey_position

    def get_self_id(self):
        return self.robot_id

    def get_target_position(self):
        n_robots = 6  # TODO:REWRITE
        radius = 2.0
        angle_increment = 2 * math.pi / n_robots
        for i in range(n_robots):
            angle = i * angle_increment
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            if np.linalg.norm(self.init_position - np.array([x, y])) > 3.99:
                return np.array([x, y])


def set_current_robot_id(robot_id):
    thread_local.robot_id = robot_id
    if robot_id not in robot_nodes:
        robot_nodes[robot_id] = RobotNode(robot_id)
    robot_nodes[robot_id].initialize_ros_node()


def get_current_robot_node():
    robot_id = getattr(thread_local, 'robot_id', None)
    if robot_id is None:
        raise ValueError("No robot_id is set for the current thread")
    return robot_nodes[robot_id]


def initialize_ros_node(robot_id):
    set_current_robot_id(robot_id)


def get_position():
    return get_current_robot_node().get_position()


def get_velocity():
    return get_current_robot_node().get_velocity()


def get_radius():
    return get_current_robot_node().get_radius()


def set_velocity(velocity):
    get_current_robot_node().set_velocity(velocity)


def get_surrounding_robots_info():
    return get_current_robot_node().get_surrounding_robots_info()


def get_surrounding_obstacles_info():
    return get_current_robot_node().get_surrounding_obstacles_info()


def get_prey_position():
    return get_current_robot_node().get_prey_position()


def get_self_id():
    return get_current_robot_node().get_self_id()


def get_target_position():
    return get_current_robot_node().get_target_position()

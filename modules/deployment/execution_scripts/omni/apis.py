import numpy as np
import rospy
from geometry_msgs.msg import Twist
from code_llm.msg import Observations

robot_info = {
    "id": None,
    "position": np.array([0.0, 0.0]),
    "radius": 0.0,
    "velocity": np.array([0.0, 0.0]),
}

ros_initialized = False
velocity_publisher: rospy.Publisher
init_position = None
target_position = None
obstacles_info = []
other_robots_info = []
prey_positions = []
moveable_objects = []
formation_points = []


def observation_callback(msg: Observations):
    global robot_info, obstacles_info, other_robots_info, prey_positions, moveable_objects, init_position, target_position
    obstacles_info = []
    other_robots_info = []
    prey_positions = []
    moveable_objects = []
    for obj in msg.observations:
        if obj.type == "Robot":
            if obj.id == robot_info["id"]:
                robot_info["position"] = np.array([obj.position.x, obj.position.y])
                if init_position is None:
                    init_position = robot_info["position"]
                robot_info["radius"] = obj.radius
                if obj.target_position is not None:
                    target_position = np.array([obj.target_position.x, obj.target_position.y])
                continue
            other_robots_info.append(
                {
                    "id": obj.id,
                    "position": np.array([obj.position.x, obj.position.y]),
                    "velocity": np.array([obj.velocity.linear.x, obj.velocity.linear.y]),
                    "radius": obj.radius,
                }
            )
        elif obj.type == "Obstacle":
            obstacles_info.append(
                {
                    "id": obj.id,
                    "position": np.array([obj.position.x, obj.position.y]),
                    "radius": obj.radius,
                }
            )


def initialize_ros_node(robot_id):
    global ros_initialized, velocity_publisher, target_position, formation_points
    robot_info["id"] = robot_id
    if not ros_initialized:
        ros_initialized = True

        rospy.Subscriber("/observation", Observations, observation_callback)
        velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        print(f"Waiting for position message from /observation...")
        msg = rospy.wait_for_message("/observation", Observations)
        print(f"Observations data init successfully")
        observation_callback(msg)
        rospy.Timer(rospy.Duration(0.1), publish_velocities)


def publish_velocities(event):
    velocity_msg = Twist()
    velocity_msg.linear.x = robot_info["velocity"][0]
    velocity_msg.linear.y = robot_info["velocity"][1]
    velocity_publisher.publish(velocity_msg)
    print(f"Publishing velocity: {robot_info['velocity']}")


def get_self_position():
    return robot_info["position"]


def get_self_velocity():
    return robot_info["velocity"]


def get_self_radius():
    return robot_info["radius"]


def set_self_velocity(velocity):
    robot_info["velocity"] = np.array(velocity)


def get_surrounding_robots_info():
    return other_robots_info


def get_surrounding_obstacles_info():
    return obstacles_info


def get_prey_position():
    return prey_positions[0] if prey_positions else None


def get_self_id():
    return robot_info["id"]


def get_target_position():
    return target_position


def get_target_formation_points():
    return formation_points

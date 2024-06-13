import geometry_msgs

import numpy as np

import rospy
from code_llm.msg import Observations, ObjInfo
from code_llm.srv import GetTargetPositions, GetTargetPositionsResponse, GetCharPointsResponse, GetCharPoints
from geometry_msgs.msg import Twist, Vector3, Point

from modules.env.char_points_generate import validate_contour_points
from modules.env.entity import Robot, Leader
from modules.env.env.env import EnvironmentBase


class Manager:
    def __init__(self, env: EnvironmentBase):
        self.env = env

        rospy.init_node('simulation_manager', anonymous=True)
        rospy.Subscriber("/leader/velocity", Twist, self.leader_velocity_callback)
        # self._reset_service = rospy.Service(
        #     "/reset_environment", SetBool, self.reset_environment_callback
        # )
        self.observation_publisher = rospy.Publisher(f"observation", Observations, queue_size=1)

        self._pub_list = []
        self._robots = env.get_entities_by_type(Robot)
        robot_start_index = self._robots[0].id
        robot_end_index = self._robots[-1].id
        rospy.set_param("robot_start_index", robot_start_index)
        rospy.set_param("robot_end_index", robot_end_index)
        for i in range(robot_start_index, robot_end_index + 1):
            rospy.Subscriber(
                f"/robot_{i}/velocity", Twist, self.velocity_callback, callback_args=i
            )
        self._target_positions_service = rospy.Service(
            "/get_target_positions", GetTargetPositions, self.get_target_positions_callback
        )
        self._char_points_service = rospy.Service(
            "/get_char_points", GetCharPoints, self.get_char_points_callback
        )
        # self._timer = rospy.Timer(rospy.Duration(0.01), self.publish_observations)
        self.received_velocity = {robot.id: False for robot in self._robots}

    def velocity_callback(self, data: geometry_msgs.msg.Twist, i):
        """
        velocity_callback is a callback function for the velocity topic.
        """
        self.env.set_entity_velocity(entity_id=i, new_velocity=np.array([data.linear.x, data.linear.y]) * 100)
        # print(f"Robot {i} velocity: {data.linear.x}, {data.linear.y}")
        self.received_velocity[i] = True  # 更新机器人接收状态

        # 检查所有机器人是否都已接收到速度信息
        if all(self.received_velocity.values()):
            print("All robots have received initial velocity. Initialization successful.")

    def leader_velocity_callback(self, data: Twist):
        leader = self.env.get_entities_by_type(Leader)[0]
        leader.speed = np.array([data.linear.x, data.linear.y])

    def connect_entities(self, entity1_id, entity2_id):
        self.env.connect_entities(entity1_id, entity2_id)

    def disconnect_entities(self, entity1_id, entity2_id):
        self.env.disconnect_entities(entity1_id, entity2_id)

    def publish_observations(self):
        observation = self.env.get_observation()
        observations_msg = Observations()
        observations_msg.observations = []
        for entity_id, entity in observation.items():
            obj_info = ObjInfo()
            obj_info.id = entity_id
            obj_info.position = Point(x=entity["position"][0], y=entity["position"][1], z=0)
            obj_info.velocity = Twist(linear=Vector3(x=entity["velocity"][0], y=entity["velocity"][1], z=0),
                                      angular=Vector3(x=0, y=0, z=0))
            obj_info.radius = entity["size"] if isinstance(entity["size"], float) else 0.0
            obj_info.type = entity["type"]
            observations_msg.observations.append(obj_info)
        self.observation_publisher.publish(observations_msg)

    def get_target_positions_callback(self, request):
        response = GetTargetPositionsResponse()
        for robot in self._robots:
            obj_info = ObjInfo()
            obj_info.id = robot.id
            if robot.target_position is None:
                robot.target_position = np.array([0, 0])
            obj_info.position = Point(x=robot.target_position[0], y=robot.target_position[1], z=0)
            response.target_positions.append(obj_info)
        return response

    def get_char_points_callback(self, request):
        char = request.character
        sampled_points = validate_contour_points(char)

        # Convert points to ROS Point message
        response = GetCharPointsResponse()
        for point in sampled_points:
            pt = Point(x=int(point[0]), y=int(point[1]), z=0)
            response.points.append(pt)

        return response

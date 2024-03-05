import numpy as np
import rospy
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float32MultiArray
from code_llm.msg import Observations, RobotInfo
from robot import Robots


class Manager:

    def __init__(self, n_robots, size):
        self._robots = Robots(n_robots, size)
        self._agent_num = n_robots
        self._position_sub = rospy.Subscriber('/robots/position', Float32MultiArray, self.position_callback)
        self._velocity_sub = rospy.Subscriber('/robots/velocity', Float32MultiArray, self.velocity_callback)
        self._pub_list = []
        self._observations_msg = Observations()
        for i in range(self._agent_num):
            self._pub_list.append(rospy.Publisher(f'/robot_{i}/observation', Observations, queue_size=1))
        self._timer = rospy.Timer(rospy.Duration(0.01), self.distribute)

    @property
    def robots(self):
        return self._robots

    def position_callback(self, data):
        """
        position_callback is a callback function for the position topic.
        """
        self._robots.positions = np.array(data.data).reshape(-1, 2)

    def velocity_callback(self, data):
        """
        velocity_callback is a callback function for the velocity topic.
        """
        self._robots.velocities = np.array(data.data).reshape(-1, 2)

    def distribute(self, event):
        """
        distribute is responsible for distributing the observations to the robots.
        """
        for i, robot in enumerate(self._robots.robots):
            self._observations_msg.observations = []
            for j, robot_j in enumerate(self._robots.robots):
                if i != j:
                    distance = np.linalg.norm(robot.position - robot_j.position)
                    if distance <= robot.communication_range:
                        robot_info = RobotInfo()
                        robot_info.id = robot_j.id
                        robot_info.position = Point(x=robot_j.position[0], y=robot_j.position[1], z=0)
                        robot_info.velocity = Vector3(x=robot_j.velocity[0], y=robot_j.velocity[1], z=0)
                        self._observations_msg.observations.append(robot_info)
            self._pub_list[i].publish(self._observations_msg)

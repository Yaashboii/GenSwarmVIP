import geometry_msgs.msg
import numpy as np
import rospy
from geometry_msgs.msg import Point, Vector3, Twist
from code_llm.msg import Observations, RobotInfo
from robot import Robots


class Manager:

    def __init__(self, n_robots, size):
        self._robots = Robots(n_robots, size)
        self._agent_num = n_robots
        self._pub_list = []
        for i in range(self._agent_num):
            self._pub_list.append(rospy.Publisher(f'/robot_{i}/observation', Observations, queue_size=1))
            rospy.Subscriber(f'/robot_{i}/velocity', Twist, self.velocity_callback, callback_args=i)

        self._timer = rospy.Timer(rospy.Duration(0.01), self.distribute)

    @property
    def robots(self):
        return self._robots

    def velocity_callback(self, data: geometry_msgs.msg.Twist, i):
        """
        velocity_callback is a callback function for the velocity topic.
        """
        self._robots.robots[i].velocity = np.array([data.linear.x, data.linear.y])

    def distribute(self, event):
        """
        distribute is responsible for distributing the observations to the robots.
        """
        for i, robot in enumerate(self._robots.robots):
            observations_msg = Observations()
            observations_msg.observations = []
            observations_msg.position = Point(x=robot.position[0], y=robot.position[1], z=0)

            observations_msg.observations = [
                RobotInfo(
                    id=robot_j.id,
                    position=Point(x=robot_j.position[0], y=robot_j.position[1], z=0),
                    velocity=Twist(
                        linear=Vector3(x=robot_j.velocity[0], y=robot_j.velocity[1], z=0),
                        angular=Vector3(x=0, y=0, z=0)
                    )
                )
                for j, robot_j in enumerate(self._robots.robots) if
                i != j and np.linalg.norm(robot.position - robot_j.position) <= robot.communication_range
            ]

            self._pub_list[i].publish(observations_msg)

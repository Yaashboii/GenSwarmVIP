import os

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from code_llm.msg import Observations

ros_initialized = False
velocity_publisher: rospy.Publisher
robot_velocity: np.ndarray
position: np.ndarray
observations: Observations
timer: rospy.Timer
robot_id: int
robots_num: int


def observation_callback(msg: Observations):
    global robot_observation, position
    robot_observation = msg.observations
    position = np.array([msg.position.x, msg.position.y])


def initialize_ros_node():
    global ros_initialized, velocity_publisher, robot_velocity, timer, position, robot_id, observations
    # avoid multiple initialization
    if not ros_initialized:
        # init ros node
        robot_id = int(os.environ['ROBOT_ID'])
        rospy.init_node(f'robot{robot_id}_control_node', anonymous=True)
        ros_initialized = True

        rospy.Subscriber(f'/robot_{robot_id}/observation', Observations, observation_callback)

        velocity_publisher = rospy.Publisher(f'/robot_{robot_id}/velocity', Twist, queue_size=1)

        # TODO: 每个节点都会set一遍，但是这个路径现在是一样的，可以优化。如果后续考虑每个机器人的数据单独保存，这一段代码可以保留
        current_folder = os.path.dirname(os.path.abspath(__file__))
        rospy.set_param('data_path', str(current_folder) + '/data')

        # make sure the position is received
        print(f"Waiting for position message from /robot_{robot_id}/observation...")
        msg = rospy.wait_for_message(f'/robot_{robot_id}/observation', Observations)
        position = np.array([msg.position.x, msg.position.y])
        robot_velocity = np.array([0.0, 0.0])

        print(f"Observations data init successfully")

        # timer to publish velocity in a fixed frequency of 100Hz
        timer = rospy.Timer(rospy.Duration(0.01), publish_velocities)


def publish_velocities(event):
    global robot_velocity, velocity_publisher
    velocity_msg = Twist()
    velocity_msg.linear.x = robot_velocity[0]
    velocity_msg.linear.y = robot_velocity[1]
    velocity_publisher.publish(velocity_msg)


def get_position():
    """
    Get the position of the robot.
    Returns:
    - numpy.ndarray: The position of the robot.

    """
    initialize_ros_node()
    global position
    return position


def get_velocity():
    """
    Get the velocity of the robot.
    Returns:
    - numpy.ndarray: The velocity of the robot.
    """
    initialize_ros_node()
    global robot_velocity
    return robot_velocity


def set_velocity(velocity):
    """
    Set the velocity of the robot.

    Parameters:
    - velocity (numpy.ndarray): The new velocity to set.
    """
    global robot_velocity
    initialize_ros_node()
    robot_velocity = np.array(velocity)


def gather_field_view_data():
    """
    Get the other robots' positions and velocities within the field of view.

    Returns:
    - A list of dictionaries, each containing:
      - 'position': A numpy array representing the robot's 2D position
       (x, y coordinates).
      - 'velocity': A numpy array representing the robot's 2D velocity (x, y components).
    if the robot is not able to observe any other robots, an empty list is returned.
    """
    global observations

    initialize_ros_node()
    observations_list = []
    for robot_info in robot_observation:
        position_array = np.array([robot_info.position.x, robot_info.position.y])
        velocity_array = np.array([robot_info.velocity.linear.x, robot_info.velocity.linear.y])
        observations_list.append({'position': position_array, 'velocity': velocity_array})

    return observations_list

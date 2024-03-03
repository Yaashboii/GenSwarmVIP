import os

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

ros_initialized = False
velocity_publisher: rospy.Publisher
robots_num: int
robots_velocity: np.ndarray
robots_position: np.ndarray
timer: rospy.Timer


def position_callback(msg):
    global robots_position
    robots_position = np.array(msg.data).reshape(-1, 2)


def initialize_ros_node():
    global ros_initialized, velocity_publisher, robots_num, robots_velocity, timer, robots_position
    if not ros_initialized:
        rospy.init_node('robot_control_node', anonymous=True)
        ros_initialized = True
        velocity_publisher = rospy.Publisher('/robots/velocity', Float32MultiArray, queue_size=10)
        rospy.Subscriber('/robots/position', Float32MultiArray, position_callback)
        robots_position = np.array(rospy.wait_for_message('/robots/position', Float32MultiArray).data).reshape(-1, 2)
        robots_num = rospy.get_param('/robots_num')
        current_folder = os.path.dirname(os.path.abspath(__file__))
        rospy.set_param('data_path', str(current_folder) + '/data')
        robots_velocity = np.zeros((robots_num, 2), dtype=float)
        timer = rospy.Timer(rospy.Duration(0.01), publish_all_velocities)


def publish_all_velocities(event):
    global robots_velocity, velocity_publisher
    velocity_msg = Float32MultiArray(data=robots_velocity.flatten().tolist())
    velocity_publisher.publish(velocity_msg)


def get_robot_position_by_id(robot_id):
    """
    Get the position of a robot with the given ID.

    Parameters:
    - robot_id (int): The ID of the robot to retrieve.

    Returns:
    - numpy.ndarray: The position of the robot.
    """
    global robots_position
    initialize_ros_node()
    positions = robots_position
    if robot_id >= positions.shape[0]:
        raise ValueError(f"Robot with ID {robot_id} not found")
    return positions[robot_id]


def set_robot_velocity_by_id(robot_id, velocity):
    """
    Set the velocity of a robot with the given ID.

    Parameters:
    - robot_id (int): The ID of the robot to set the velocity for.
    - velocity (numpy.ndarray): The new velocity to set.
    """
    initialize_ros_node()  # 确保ROS节点已初始化
    velocity = np.array(velocity, dtype=float)
    if robot_id >= robots_num:
        raise ValueError(f"Robot with ID {robot_id} not found")
    if velocity.shape != (2,):
        raise ValueError(f"Expected velocity shape (2,), got {velocity.shape}")
    robots_velocity[robot_id] = velocity


def get_all_robot_ids():
    """
    Get a list of all existing robot IDs.

    Returns:
    - list: A list containing all the robot IDs.
    """
    global robots_num
    initialize_ros_node()

    return list(range(robots_num))


def get_robots_count():
    """
    Get the total number of robots.

    Returns:
    - int: The total number of robots.
    """
    global robots_num
    initialize_ros_node()
    return robots_num


def get_leader_position():
    """
    Get the position of the leader robot.

    Returns:
    - numpy.ndarray: The position of the leader robot.
    """
    initialize_ros_node()
    position_msg = rospy.wait_for_message('/leader/position', Float32MultiArray)
    return np.array(position_msg.data)

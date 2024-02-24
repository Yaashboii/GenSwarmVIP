import numpy as np

import rospy
from std_msgs.msg import Float32MultiArray
from robot import robots

ros_initialized = False
velocity_publishers = {}


def initialize_ros_node():
    global ros_initialized, velocity_publishers
    if not ros_initialized:
        rospy.init_node('robot_control_node', anonymous=True)
        ros_initialized = True
        robot_ids = get_all_robot_ids()
        for robot_id in robot_ids:
            velocity_publishers[robot_id] = rospy.Publisher(f'/robot_{robot_id}/velocity', Float32MultiArray,
                                                            queue_size=10)


def get_robot_position_by_id(robot_id):
    """
    Get the position of a robot with the given ID using ROS topic subscription.

    Parameters:
    - robot_id (int): The ID of the robot to retrieve.

    Returns:
    - numpy.ndarray: The position of the robot.

    Raises:
    - ValueError: If no robot with the specified ID is found.
    """
    initialize_ros_node()
    for robot in robots:
        if robot.robot_id == robot_id:
            position_msg = rospy.wait_for_message(f'/robot_{robot_id}/position', Float32MultiArray)
            return np.array(position_msg.data)
    raise ValueError(f"Robot with ID {robot_id} not found")


def set_robot_velocity_by_id(robot_id, velocity):
    """
    Set the velocity of a robot with the given ID using ROS topic publication.

    Parameters:
    - robot_id (int): The ID of the robot to set the velocity for.
    - velocity (numpy.ndarray): The new velocity to set.

    Raises:
    - ValueError: If no robot with the specified ID is found.
    """
    initialize_ros_node()  # 确保ROS节点已初始化
    for robot in robots:
        if robot.robot_id == robot_id:
            # 使用ROS发布机器人速度
            velocity_msg = Float32MultiArray(data=velocity.tolist())
            velocity_publisher = rospy.Publisher(f'/robot_{robot_id}/velocity', Float32MultiArray, queue_size=10)
            velocity_publisher.publish(velocity_msg)
            return
    raise ValueError(f"Robot with ID {robot_id} not found")


def get_all_robot_ids():
    """
    Get a list of all existing robot IDs.

    Returns:
    - list: A list containing all the robot IDs.
    """
    return [robot.robot_id for robot in robots]

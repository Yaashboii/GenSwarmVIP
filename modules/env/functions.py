import os

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from code_llm.msg import Observations

ros_initialized = False
velocity_publisher: rospy.Publisher
robots_velocity: np.ndarray
robots_position: np.ndarray
robot_observation: Observations
timer: rospy.Timer
robot_id: int
robots_num: int


def position_callback(msg):
    global robots_position
    robots_position = np.array(msg.data).reshape(-1, 2)


def observation_callback(msg: Observations):
    global robot_observation
    robot_observation = msg.observations


def initialize_ros_node():
    global ros_initialized, velocity_publisher, robots_num, robots_velocity, timer, robots_position, robot_id, robot_observation
    if not ros_initialized:
        robot_id = int(os.environ['ROBOT_ID'])
        rospy.init_node('robot_control_node', anonymous=True)
        ros_initialized = True

        velocity_publisher = rospy.Publisher('/robots/velocity', Float32MultiArray, queue_size=10)
        current_folder = os.path.dirname(os.path.abspath(__file__))
        rospy.set_param('data_path', str(current_folder) + '/data')
        rospy.Subscriber('/robots/position', Float32MultiArray, position_callback)
        robots_position = np.array(rospy.wait_for_message('/robots/position',
                                                          Float32MultiArray).data).reshape(-1, 2)
        robots_num = rospy.get_param('/robots_num')
        robot_observation = []
        rospy.Subscriber(f'/robot_{robot_id}/observation', Observations, observation_callback)
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
    velocity = np.array(velocity, dtype=float)
    if robot_id >= robots_num:
        raise ValueError(f"Robot with ID {robot_id} not found")
    if velocity.shape != (2,):
        raise ValueError(f"Expected velocity shape (2,), got {velocity.shape}")
    robots_velocity[robot_id] = velocity


def get_position():
    """
    Get the position of the robot.
    Returns:
    - numpy.ndarray: The position of the robot.

    """
    initialize_ros_node()

    return get_robot_position_by_id(robot_id)


def set_velocity(velocity):
    """
    Set the velocity of the robot.

    Parameters:
    - velocity (numpy.ndarray): The new velocity to set.
    """
    initialize_ros_node()

    set_robot_velocity_by_id(robot_id, velocity)


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
    global robot_observation

    initialize_ros_node()
    observations_list = []
    for robot_info in robot_observation:
        position_array = np.array([robot_info.position.x, robot_info.position.y])
        velocity_array = np.array([robot_info.velocity.x, robot_info.velocity.y])
        observations_list.append({'position': position_array, 'velocity': velocity_array})

    return observations_list


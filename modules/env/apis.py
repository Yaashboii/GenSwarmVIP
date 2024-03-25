import os
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from code_llm.msg import Observations

ros_initialized = False
velocity_publisher: rospy.Publisher
robot_info: dict = {
    'position': np.array([0.0, 0.0]),
    'radius': 0.0,
    'velocity': np.array([0.0, 0.0])
}
timer: rospy.Timer

obstacles_info: list[dict] = []
other_robots_info: list[dict] = []
robot_id: int


def observation_callback(msg: Observations):
    global obstacles_info, robot_info, other_robots_info
    obstacles_info = []
    other_robots_info = []
    for obj in msg.observations:
        if obj.type == 'self':
            self_info = obj
            robot_info['position'] = np.array([self_info.position.x, self_info.position.y])
            robot_info['radius'] = self_info.radius
            robot_info['velocity'] = np.array([0.0, 0.0])
        elif obj.type == 'robot':
            other_robots_info.append(
                {
                    'position': np.array([obj.position.x, obj.position.y]),
                    'velocity': np.array([obj.velocity.linear.x, obj.velocity.linear.y]),
                    'radius': obj.radius,
                }
            )
        elif obj.type == 'obstacle':
            obstacles_info.append(
                {
                    'position': np.array([obj.position.x, obj.position.y]),
                    'radius': obj.radius
                }
            )


def initialize_ros_node():
    global ros_initialized, velocity_publisher, timer, robot_id, robot_info
    # avoid multiple initialization
    if not ros_initialized:
        # init ros node
        from run import robot_id
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
        observation_callback(msg)
        print(f"Observations data init successfully")

        # timer to publish velocity in a fixed frequency of 100Hz
        timer = rospy.Timer(rospy.Duration(0.01), publish_velocities)


def publish_velocities(event):
    global robot_info, velocity_publisher
    velocity_msg = Twist()
    velocity_msg.linear.x = robot_info['velocity'][0]
    velocity_msg.linear.y = robot_info['velocity'][1]
    velocity_publisher.publish(velocity_msg)


def get_position():
    """
    Get the position of the robot.
    Returns:
    - numpy.ndarray: The position of the robot.

    """
    initialize_ros_node()
    global robot_info
    return robot_info['position']


def get_radius():
    """
    Get the radius of the robot.
    Returns:
    - float: The radius of the robot.
    """
    initialize_ros_node()
    global robot_info
    return robot_info['radius']


def get_velocity():
    """
    Get the velocity of the robot.
    Returns:
    - numpy.ndarray: The velocity of the robot.
    """
    initialize_ros_node()
    global robot_info
    return robot_info['velocity']


def set_velocity(velocity):
    """
    Set the velocity of the robot.

    Parameters:
    - velocity (numpy.ndarray): The new velocity to set.
    """
    global robot_info
    initialize_ros_node()
    robot_info['velocity'] = np.array(velocity)


def get_surrounding_robots_info():
    """
    Get the information of the surrounding robots.
    Returns:
    - list: A list of dictionaries, each containing the position, velocity, and radius of a robot.
        - position (numpy.ndarray): The position of the robot.
        - velocity (numpy.ndarray): The velocity of the robot.
        - radius (float): The radius of the robot.
    """
    global other_robots_info
    initialize_ros_node()
    return other_robots_info


def get_surrounding_obstacles_info():
    """
    Get the information of the surrounding obstacles.
    Returns:
    - list: A list of dictionaries, each containing the position and radius of an obstacle.
        - position (numpy.ndarray): The position of the obstacle.
        - radius (float): The radius of the obstacle.
    """
    global obstacles_info
    initialize_ros_node()
    return obstacles_info

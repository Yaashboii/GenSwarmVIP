ROBOT_API = """
def get_position():
    '''
    Get the position of the robot.
    Returns:
    - numpy.ndarray: The position of the robot.
    '''


def get_radius():
    '''
    Get the radius of the robot.
    Returns:
    - float: The radius of the robot.
    '''


def get_velocity():
    '''
    Get the velocity of the robot.
    Returns:
    - numpy.ndarray: The velocity of the robot.
    '''


def set_velocity(velocity):
    '''
    Set the velocity of the robot.

    Parameters:
    - velocity (numpy.ndarray): The new velocity to set.
    '''

def get_surrounding_robots_info():
    '''
    Get the information of the surrounding robots.
    Returns:
    - list: A list of dictionaries, each containing the position, velocity, and radius of a robot.
        - position (numpy.ndarray): The position of the robot.
        - velocity (numpy.ndarray): The velocity of the robot.
        - radius (float): The radius of the robot.
    '''


def get_surrounding_obstacles_info():
    '''
    Get the information of the surrounding obstacles.
    Returns:
    - list: A list of dictionaries, each containing the position and radius of an obstacle.
        - position (numpy.ndarray): The position of the obstacle.
        - radius (float): The radius of the obstacle.
    '''
""".strip()

LEADER_API = """
def get_leader_position():
    '''
    Get the position of the leader robot.

    Returns:
    - numpy.ndarray: The position of the leader robot.
    '''
""".strip()


def get_robot_api(leader=False):
    if leader:
        return ROBOT_API + LEADER_API

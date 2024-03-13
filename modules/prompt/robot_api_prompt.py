ROBOT_API = """
def get_radius():
    '''
    Description: Get the radius of the robot itself.
    Returns:
    - float: The radius of the robot itself.
    Usage:
    radius = get_radius()
    '''

def get_position():
    '''
    Description: Get the position of robot itself.
    Returns:
    - numpy.ndarray: The position of the robot itself.
    Usage:
    pos = get_position()
    x, y = pos[0], pos[1]
    '''

def set_velocity(velocity):
    '''
    Description: Set the velocity of the robot itself
    Input:
    - velocity (numpy.ndarray): The new velocity to set.
    Usage: 
    velocity = np.array([vx, vy])
    set_velocity(velocity)
    '''

def get_velocity():
    '''
    Get the velocity of the robot itself
    Returns:
    - numpy.ndarray: The velocity of the robot.
    Usage:
    vel = get_velocity()
    '''

def get_surrounding_robots_info():
    '''
    Get the information of the surrounding robots.
    Returns:
    - list: A list of dictionaries, each containing the position, velocity, and radius of a robot.
        - position (numpy.ndarray): The position of the robot.
        - velocity (numpy.ndarray): The velocity of the robot.
        - radius (float): The radius of the robot.
    Usage:
    robots_info = get_surrounding_robots_info()
    for robot_info in robots_info:
        pos = robot_info['position']
        vel = robot_info['velocity']
        radius = robot_info['radius']
    '''


def get_surrounding_obstacles_info():
    '''
    Get the information of the surrounding obstacles.
    Returns:
    - list: A list of dictionaries, each containing the position and radius of an obstacle.
        - position (numpy.ndarray): The position of the obstacle.
        - radius (float): The radius of the obstacle.
    Usage:
    obstacles_info = get_surrounding_obstacles_info()
    for obstacle_info in obstacles_info:
        pos = obstacle_info['position']
        radius = obstacle_info['radius']
    '''
""".strip()


def get_robot_api(leader=False):
    if leader:
        return ROBOT_API

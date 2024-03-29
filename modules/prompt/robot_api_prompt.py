from modules.utils import extract_imports_and_functions, extract_top_level_function_names

robot_api = """
def get_position():
    '''
    Description: Get the current position of the robot itself in real-time.
    Returns:
    - numpy.ndarray: The current, real-time position of the robot.
    '''

def set_velocity(velocity):
    '''
    Description: Set the velocity of the robot itself in real-time.
    Input:
    - velocity (numpy.ndarray): The new velocity to be set immediately.
    '''

def get_velocity():
    '''
    Get the current velocity of the robot itself in real-time.
    Returns:
    - numpy.ndarray: The current, real-time velocity of the robot.
    '''

def get_surrounding_robots_info():
    '''
    Get real-time information of the surrounding robots.
    Returns:
    - list: A list of dictionaries, each containing the current position, velocity, and radius of a robot, reflecting real-time data.
        - position (numpy.ndarray): The current position of the robot.
        - velocity (numpy.ndarray): The current velocity of the robot.
        - radius (float): The radius of the robot.
    '''

def get_surrounding_obstacles_info():
    '''
    Get real-time information of the surrounding obstacles.
    Returns:
    - list: A list of dictionaries, each containing the current position and radius of an obstacle, reflecting real-time data.
        - position (numpy.ndarray): The current position of the obstacle.
        - radius (float): The radius of the obstacle.
    '''
""".strip()


class RobotApi:
    def __init__(self, content):
        self.content = content
        _, api_list = extract_imports_and_functions(content)
        self.apis = {}
        for api in api_list:
            name = extract_top_level_function_names(api)[0]
            self.apis[name] = api

    def get_prompt(self, name: list[str] | str = None) -> str:
        if isinstance(name, str):
            name = [name]
        if name is None:
            return '\n\n'.join(self.apis.values())
        try:
            prompts = [self.apis[n] for n in name]
            return '\n\n'.join(prompts)
        except Exception as e:
            raise SystemExit(f"Error in get_prompt: {e},current existing apis:{self.apis.keys()},input name:{name}")


robot_api = RobotApi(content=robot_api)
ROBOT_API = robot_api.get_prompt()

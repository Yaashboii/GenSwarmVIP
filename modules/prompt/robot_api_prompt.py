from modules.framework.code.parser import _AstParser
from modules.utils import extract_function_definitions,

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
    Note: This API updates at a fixed rate of 10Hz to the vehicle, so there's no need to use time.sleep to limit the frequency, as the provided API has already implemented this.
    Input:
    - velocity (numpy.ndarray): The new velocity to be set immediately.
    '''

def get_velocity():
    '''
    Get the current velocity of the robot itself in real-time.
    Returns:
    - numpy.ndarray: The current, real-time velocity of the robot.
    '''

def get_radius():
    '''
    Description: Get the radius of the robot itself.
    Returns:
    - float: The radius of the robot itself.
    '''

def get_surrounding_robots_info():
    '''
    Get real-time information of the surrounding robots. The data provided by this function are all within the robot's sensory range.
    Note: This API is provided by humans, and the perception data it offers are results within a 5m radius centered on the robot's position. There is no need to concern yourself with how it is implemented; you only need to call it.
    Returns:
    - list: A list of dictionaries, each containing the current position, velocity, and radius of a robot, reflecting real-time data.
        - position (numpy.ndarray): The current position of the robot.
        - velocity (numpy.ndarray): The current velocity of the robot.
        - radius (float): The radius of the robot.
    '''

def get_surrounding_obstacles_info():
    '''
    Get real-time information of the surrounding obstacles. The data provided by this function are all within the robot's sensory range.
    Note: This API is provided by humans, and the perception data it offers are results within a 5m radius centered on the robot's position. There is no need to concern yourself with how it is implemented; you only need to call it.
    Returns:
    - list: A list of dictionaries, each containing the current position and radius of an obstacle, reflecting real-time data.
        - position (numpy.ndarray): The current position of the obstacle.
        - radius (float): The radius of the obstacle.
    '''
""".strip()


# data_api = read_file("modules/env/", filename="data_apis.py")

class RobotApi:
    def __init__(self, content):
        self.content = content
        code_obj = _AstParser(content)
        self.apis = {}
        for api in code_obj.function_defs:
            code_obj = _AstParser(api)
            name = code_obj.function_names()[0]
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
# data_api = RobotApi(content=data_api)
# DATA_API = data_api.get_prompt()

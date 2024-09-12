import os
import yaml

from modules.framework.parser import CodeParser

robot_api_prompt = """
def get_self_id():
    '''
    Description: Get the unique ID of the robot itself.
    Returns:
    - int: The unique ID of the robot itself.ID is a unique identifier for each robot in the environment.
    '''
    
def get_all_robots_info():
    '''
    Get real-time information of all the robots id information in the environment.
    Returns:
    - dict: 
        - self_id (int): The unique ID of the robot itself.
        - start_id (int): The unique ID of the first robot in the environment.
        - end_id (int): The unique ID of the last robot in the environment.
        - robots_num (int): The number of robots in the environment.
    '''
    
def get_self_position():
    '''
    Description: Get the current position of the robot itself in real-time.
    Returns:
    - numpy.ndarray: The current, real-time position of the robot.
    '''
    
def set_self_velocity(velocity):
    '''
    Description: Set the velocity of the robot itself in real-time.
    Note: This API updates at a fixed rate of 10Hz to the vehicle, so there's no need to use time.sleep to limit the frequency, as the provided API has already implemented this.
    Input:
    - velocity (numpy.ndarray): The new velocity to be set immediately.
    '''
    
def get_self_velocity():
    '''
    Get the current velocity of the robot itself in real-time.
    Returns:
    - numpy.ndarray: The current, real-time velocity of the robot.
    '''
def get_self_radius():
    '''
    Description: Get the radius of the robot itself.Radius is the distance from the center of the robot to the edge of the robot.
    It is used to judge whether the robot is in collision with other robots or obstacles by comparing the distance between the centers of the robots or obstacles and the sum of their radii.
    Returns:
    - float: The radius of the robot itself.
    '''
    
def get_surrounding_robots_info():
    '''
    Get real-time information of the surrounding robots.
    Note: This API is provided by humans. There is no need to concern yourself with how it is implemented; you only need to call it.
    Returns:
    - list: A list of dictionaries, each containing the current position, velocity, and radius of a robot, reflecting real-time data.
        - id (int): The unique ID of the robot.
        - position (numpy.ndarray): The current position of the robot.
        - velocity (numpy.ndarray): The current velocity of the robot.
        - radius (float): The radius of the robot.
    '''
def get_surrounding_obstacles_info():
    '''
    Get real-time information of the surrounding obstacles. The data provided by this function are all within the robot's sensory range.
    Note: This API is provided by humans. There is no need to concern yourself with how it is implemented; you only need to call it.
    Returns:
    - list: A list of dictionaries, each containing the current position and radius of an obstacle, reflecting real-time data.
        - id (int): The unique ID of the obstacle.
        - position (numpy.ndarray): The current position of the obstacle.
        - radius (float): The radius of the obstacle.
    '''
def get_prey_position():
    '''
    Description: Get the position of the prey.
    Returns:
    - numpy.ndarray: The position of the prey.
    '''
    
def get_target_position():
    '''
    Description: Get the target position for the robot to reach.
    Returns:
    - numpy.ndarray: The current position of the target.
    '''
    
def get_target_formation_points():
    '''
    Description: Get the target formation points for the robot to reach.
    Returns:
    - list: A list of numpy.ndarray,all of which form the target formation,each representing a target point for one robot to reach.
        - position (numpy.ndarray): The position of the target formation point.
    '''


def get_sheep_positions():
    '''
    Description: Get the positions of all the sheep in the environment.
    Returns:
    - list: A list of numpy.ndarray, each representing the position of a sheep.
        - position (numpy.ndarray): The position of a sheep.
    '''

def pick_up_object(object_id):
    '''
    Description: Pick up an object with the specified ID.
    Input:
    - object_id (int): The ID of the object to pick up.
    Returns:
    - bool: True if the object is successfully picked up, False means the object is too far away to be picked up.
    '''

def put_down_object(object_id):
    '''
    Description: Put down an object with the specified ID.
    Input:
    - object_id (int): The ID of the object to put down.
    Returns:
    - bool: True if the object is successfully put down, False means the object is not picked up.
    '''

def get_object_to_transport_info():
    '''
    Description: Get real-time information of the object that the robot needs to transport.
    Note: This API is provided by humans. There is no need to concern yourself with how it is implemented; you only need to call it.
    Returns:
    -list: A list of dictionaries, each containing the current position and radius of the object that the robot is transporting, reflecting real-time data.
        - id (int): The unique ID of the object.
        - position (numpy.ndarray): The current position of the object.
        - radius (float): The radius of the object.
        - target_position (numpy.ndarray): The target position of the object.
    '''

def connect_to_another_robot(target_id):
    '''
    Description: Connect to another robot with the specified ID.
    Input:
    - target_id (int): The ID of the robot to connect to.
    Returns:
    - bool: True if the connection is successful, False means the connection is not possible.
    '''
    
def get_unexplored_area():
    '''
    Description: Get the unexplored area in the environment.
    Returns:
    - list: A list of dictionaries, each containing the id and position of an unexplored area.
        - id (int): The unique ID of the unexplored area.
        - position (numpy.ndarray): The position of the unexplored area.
    '''

def get_quadrant_target_position():
    '''
    Description: Get the target position for the robot to reach in the quadrant.
    Returns:
    - dict: A dictionary where the keys are integers representing quadrant indices (1, 2, 3, 4) and the values are numpy arrays (np.array) containing two elements, which represent the target position in 2D coordinates (x, y) for the robot to reach within the corresponding quadrant.
    '''

""".strip()


class RobotApi:
    def __init__(self, content):
        self.content = content
        code_obj = CodeParser()
        code_obj.parse_code(self.content)
        self.apis = code_obj.function_defs

        self.base_apis = ['get_self_id',
                          'get_all_robots_info',
                          'get_self_position',
                          'set_self_velocity',
                          'get_self_velocity',
                          'get_self_radius',
                          'get_surrounding_robots_info',
                          ]
        self.base_prompt = [self.apis[api] for api in self.base_apis]
        self.task_apis = {
            "bridging": [],
            "circling": [],
            "covering": [],
            "crossing": ['get_surrounding_obstacles_info'],
            "encircling": ["get_prey_position", 'get_surrounding_obstacles_info'],
            "exploration": ['get_unexplored_area'],
            "flocking": ['get_surrounding_obstacles_info'],
            "clustering": ['get_surrounding_obstacles_info', 'get_quadrant_target_position'],
            "herding": ['get_sheep_positions', 'get_surrounding_obstacles_info'],
            "shaping": ['get_target_formation_points'],
            "formation": ['get_surrounding_obstacles_info'],
            "transportation": ['pick_up_object',
                               'put_down_object',
                               'get_object_to_transport_info'
                               ],
            "pursuing": ['get_prey_position', 'get_surrounding_obstacles_info'],

        }

    def get_prompt(self, task_name: str = None) -> str:
        """
        Get the prompt of the robot API.
        Parameters:
            task_name: str, the name of the task to get the prompt.
            default is None, which means to get all the prompts.
        Returns:
            str: The prompt of the robot API.
        """
        if task_name is None:
            return "\n\n".join(self.apis.values())
        try:
            task_prompt = self.base_prompt.copy()
            specific_apis = [self.apis[api] for api in self.task_apis[task_name]]
            task_prompt.extend(specific_apis)
            return "\n\n".join(task_prompt)
        except Exception as e:
            raise SystemExit(
                f"Error in get_prompt: {e},current existing apis:{self.apis.keys()},"
                f"input task name:{task_name},expected name:{self.task_apis[task_name]}"
            )


robot_api = RobotApi(content=robot_api_prompt)

script_dir = os.path.dirname(os.path.abspath(__file__))
yaml_file_path = os.path.join(script_dir, '../../config/', 'experiment_config.yaml')
with open(yaml_file_path, 'r', encoding='utf-8') as file:
    data = yaml.safe_load(file)
task_name = data['arguments']['--run_experiment_name']['default'][0]
ROBOT_API = robot_api.get_prompt(task_name)

if __name__ == '__main__':
    for task_name in robot_api.task_apis.keys():
        print(f"Task name: {task_name}")
        print(robot_api.get_prompt(task_name))
        print("=" * 50)
        print()

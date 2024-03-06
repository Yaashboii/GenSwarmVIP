import datetime
import threading
from pathlib import Path

ENV_DES = '''
below are the basic information of the environment and robot:
environment:
    size: 10m x 10m
    Obstacles: None
    X-axis: -5m to 5m
    Y-axis: -5m to 5m
    
Robot: 
    Max Speed: 2m/s
    Control: velocity control
    Max Control Frequency: 20Hz

'''

ROBOT_API = """
def get_position():
    '''
    Get the position of the robot.
    Returns:
    - numpy.ndarray: The position of the robot.

    '''


def set_velocity(velocity):
    '''
    Set the velocity of the robot.

    Parameters:
    - velocity (numpy.ndarray): The new velocity to set.
    '''


def get_velocity():
    '''
    Get the velocity of the robot.
    Returns:
    - numpy.ndarray: The velocity of the robot.
    '''


def gather_field_view_data():
    '''
    Get the other robots' positions and velocities within the field of view.
    Returns:
    - A list of dictionaries, each containing:
      - 'position': A numpy array representing the robot's 2D position (x, y coordinates).
      - 'velocity': A numpy array representing the robot's 2D velocity (x, y components).
    if the robot is not able to observe any other robots, an empty list is returned.
    '''
"""

LEADER_API = """
def get_leader_position():
    '''
    Get the position of the leader robot.

    Returns:
    - numpy.ndarray: The position of the leader robot.
    '''
"""


def get_robot_api(leader=False):
    if leader:
        return ROBOT_API + LEADER_API


def get_project_root():
    """Search upwards to find the project root directory."""
    current_path = Path.cwd()
    while True:
        if (
                (current_path / ".git").exists()
                or (current_path / ".project_root").exists()
                or (current_path / ".gitignore").exists()
        ):
            # use metagpt with git clone will land here
            return current_path
        parent_path = current_path.parent
        if parent_path == current_path:
            # use metagpt with pip install will land here
            cwd = Path.cwd()
            return cwd
        current_path = parent_path


current_datetime = datetime.datetime.now()
formatted_date = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
PROJECT_ROOT = get_project_root()
WORKSPACE_ROOT = PROJECT_ROOT / f"workspace/{formatted_date}"
DATA_PATH = WORKSPACE_ROOT / "data"
ENV_PATH = WORKSPACE_ROOT / "env"

GLOBAL_LOCK = threading.Lock()


def set_workspace_root(workspace_root: str):
    global WORKSPACE_ROOT, DATA_PATH, ENV_PATH

    WORKSPACE_ROOT = Path(workspace_root)

    DATA_PATH = WORKSPACE_ROOT / "data"
    ENV_PATH = WORKSPACE_ROOT / "env"

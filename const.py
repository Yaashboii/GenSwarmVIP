import datetime
from pathlib import Path
from loguru import logger

ENV_CODE = """

import numpy as np

class Robot:
    def __init__(self, robot_id, initial_position):
        self.robot_id = robot_id
        self.position = np.array(initial_position, dtype=float)
    def update_position(self, velocity, dt):
        self.position += np.array(velocity, dtype=float) * dt

class Env:
    def __init__(self):
        self.robots = []
        self.time = 0

    def create_robot(self, initial_position):
        robot_id = len(self.robots)
        new_robot = Robot(robot_id, initial_position)
        self.robots.append(new_robot)

    def get_robot_positions(self):
        return [robot.position for robot in self.robots]

    def update_environment(self, velocities, dt):
        self.time += dt
        for robot, velocity in zip(self.robots, velocities):
            robot.update_position(velocity, dt)
env = Env()
"""


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
            logger.info(f"PROJECT_ROOT set to {str(current_path)}")
            return current_path
        parent_path = current_path.parent
        if parent_path == current_path:
            # use metagpt with pip install will land here
            cwd = Path.cwd()
            logger.info(f"PROJECT_ROOT set to current working directory: {str(cwd)}")
            return cwd
        current_path = parent_path


current_datetime = datetime.datetime.now()
formatted_date = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
PROJECT_ROOT = get_project_root()
DATA_PATH = PROJECT_ROOT / "data"
WORKSPACE_ROOT = PROJECT_ROOT / f"workspace/{formatted_date}"


from .base_entity import Entity
import numpy as np


class Robot(Entity):
    def __init__(self, robot_id, initial_position, size, target_position=None, color='green'):
        super().__init__(robot_id, initial_position, size, color=color, collision=True, movable=True)

        self.__target_position = np.array(target_position, dtype=float) if target_position is not None else None

    @property
    def target_position(self):
        """Get the target position of the robot."""
        return self.__target_position

    @target_position.setter
    def target_position(self, value):
        """Set a new target position for the robot."""
        self.__target_position = value

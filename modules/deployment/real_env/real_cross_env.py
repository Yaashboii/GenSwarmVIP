import json

import numpy as np
import pygame

from modules.deployment.entity import Leader, Obstacle, Robot
from .gymnasium_real_env import GymnasiumRealEnvironment



class RealCrossEnvironment(GymnasiumRealEnvironment):
    def __init__(self,
                 data_file: str = None,
                 engine_type='OmniEngine',
                 output_file: str = "output.json"):
        super().__init__(data_file=data_file, engine_type=engine_type)
        self.output_file = output_file
        self.init_entities()

    def init_entities(self):
        robots_pos = [entity.position for entity in self.get_entities_by_type("Robot")]
        target_pos = self.find_farthest_points(robots_pos)
        for index, robot in enumerate(self.get_entities_by_type("Robot")):
            robot.target_position = target_pos[index]

    @staticmethod
    def find_farthest_points(points):
        points = np.array(points)
        distances = np.linalg.norm(points[:, np.newaxis] - points, axis=2)
        farthest_indices = np.argmax(distances, axis=1)
        return points[farthest_indices]

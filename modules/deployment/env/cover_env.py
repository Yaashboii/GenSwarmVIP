import json

import numpy as np
import pygame

from modules.deployment.entity import Entity, Robot
from modules.deployment.env import EnvironmentBase


class CoverEnvironment(EnvironmentBase):
    def __init__(self,
                 width: int,
                 height: int,
                 robot_num: int,
                 output_file: str = "output.json"):
        super().__init__(width, height)
        self.output_file = output_file
        self.robot_num = robot_num
        self.init_entities()

    def init_entities(self):
        robot_points = self.sample_points_inside_circle(200, (500, 500), self.robot_num, 10)
        for entity_id, initial_position in enumerate(robot_points):
            robot = Robot(robot_id=entity_id,
                          initial_position=initial_position,
                          target_position=None,
                          size=5.0)
            self.add_entity(robot)

    @staticmethod
    def sample_points_inside_circle(radius, center, num_points, min_distance, max_attempts_per_point=10):
        def distance(p1, p2):
            return np.sqrt(np.sum((p1 - p2) ** 2, axis=1))

        points = []
        center = np.array(center)
        attempts = 0

        radius -= min_distance
        while len(points) < num_points and attempts < num_points * max_attempts_per_point:
            # Generate random points within the bounding square
            random_points = np.random.uniform(-radius, radius, size=(num_points, 2)) + center
            valid_mask = np.linalg.norm(random_points - center, axis=1) <= radius
            random_points = random_points[valid_mask]

            for point in random_points:
                if len(points) == 0 or np.all(distance(np.array(points), point) >= min_distance):
                    points.append(point)
                    if len(points) >= num_points:
                        break

            attempts += 1

        if len(points) < num_points:
            print(f"Warning: Could only place {len(points)} points out of {num_points} requested.")

        return np.array(points)

import json

import numpy as np
import pygame

from modules.deployment.entity.entity import Entity
from modules.deployment.entity.landmark import Landmark
from modules.deployment.entity.robot import Robot
from modules.deployment.env.base_env import EnvironmentBase


class AssemblyEnvironment(EnvironmentBase):
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
        range_a = Landmark(landmark_id=self.robot_num,
                           initial_position=(100, 500),
                           size=np.array((200, 1000)),
                           color='gray')
        range_b = Landmark(landmark_id=self.robot_num + 1,
                           initial_position=(900, 500),
                           size=np.array((200, 1000)),
                           color='gray')

        self.add_entity(range_a)
        self.add_entity(range_b)
        robot_points = self.sample_points(num_points=self.robot_num,
                                          center=(500, 500),
                                          min_distance=20,
                                          shape='rectangle',
                                          size=(600, 1000))
        for entity_id, initial_position in enumerate(robot_points):
            robot = Robot(robot_id=entity_id,
                          initial_position=initial_position,
                          size=10.0)
            self.add_entity(robot)

    @staticmethod
    def sample_points(center, num_points, min_distance, shape='circle', size=None, max_attempts_per_point=10):
        def distance(p1, p2):
            return np.sqrt(np.sum((p1 - p2) ** 2, axis=1))

        points = []
        center = np.array(center)
        attempts = 0

        if shape == 'circle':
            if size is None or len(size) != 1:
                raise ValueError("For circle, size should be a list or tuple with one element: [radius].")
            radius = size[0] - min_distance
        elif shape == 'rectangle':
            if size is None or len(size) != 2:
                raise ValueError("For rectangle, size should be a list or tuple with two elements: [width, height].")
            width, height = size
            width -= 2 * min_distance
            height -= 2 * min_distance
        else:
            raise ValueError(f"Unsupported shape: {shape}")

        while len(points) < num_points and attempts < num_points * max_attempts_per_point:
            if shape == 'circle':
                # Generate random points within the bounding square and then filter to within the circle
                random_points = np.random.uniform(-radius, radius, size=(num_points, 2)) + center
                valid_mask = np.linalg.norm(random_points - center, axis=1) <= radius
                random_points = random_points[valid_mask]
            elif shape == 'rectangle':
                # Generate random points within the bounding rectangle
                random_points = np.random.uniform(-0.5, 0.5, size=(num_points, 2)) * np.array([width, height]) + center
            else:
                raise ValueError(f"Unsupported shape: {shape}")

            for point in random_points:
                if len(points) == 0 or np.all(distance(np.array(points), point) >= min_distance):
                    points.append(point)
                    if len(points) >= num_points:
                        break

            attempts += 1

        if len(points) < num_points:
            print(f"Warning: Could only place {len(points)} points out of {num_points} requested.")

        return np.array(points)

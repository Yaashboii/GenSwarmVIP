import json

import numpy as np
import pygame

from modules.env.entity import Robot, Prey, Obstacle, Landmark
# TODO:重新命名
from modules.env.env.env import EnvironmentBase


class SheepdogEnvironment(EnvironmentBase):
    def __init__(self,
                 width: int,
                 height: int,
                 sheep_num: int,
                 dog_num: int,
                 output_file: str = "output.json"):
        super().__init__(width, height)
        self.landmark = None
        self.output_file = output_file
        self.sheep_num = sheep_num
        self.dog_num = dog_num
        self.init_entities()

    def init_entities(self):
        circle = Landmark(landmark_id=self.dog_num + self.sheep_num,
                          initial_position=(500, 500),
                          size=75.0,
                          color='gray')
        self.landmark = circle
        self.add_entity(circle)
        robot_points = self.sample_points_inside_circle(450, (500, 500),
                                                        self.dog_num + self.sheep_num,
                                                        50)
        for entity_id, initial_position in enumerate(robot_points[:self.dog_num]):
            dog = Robot(robot_id=entity_id,
                        initial_position=initial_position,
                        size=10.0)
            self.add_entity(dog)
        for entity_id, initial_position in enumerate(robot_points[self.dog_num:], start=self.dog_num):
            sheep = Prey(prey_id=entity_id,
                         initial_position=initial_position,
                         size=10.0)
            sheep.max_speed = 100
            self.add_entity(sheep)

    def is_within_landmark(self, position):
        landmark_position = self.landmark.position
        distance = np.linalg.norm(np.array(position) - np.array(landmark_position))
        return distance <= self.landmark.size

    def update(self, dt: float):
        for entity in self.entities:
            if entity.__class__.__name__ == "Prey":
                if not self.is_within_landmark(entity.position):
                    entity.avoid_robots_and_walls(self.get_entities_by_type(Robot))
                else:
                    continue
            entity.move(dt)

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

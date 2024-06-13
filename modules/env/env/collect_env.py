import json

import numpy as np
import pygame

from modules.env.entity import Robot, Prey, Obstacle, Landmark, PushableObject
# TODO:重新命名
from modules.env.env.env import EnvironmentBase


class CollectEnvironment(EnvironmentBase):
    def __init__(self,
                 width: int,
                 height: int,
                 num_1: int,
                 num_2: int,
                 robot_num: int,
                 output_file: str = "output.json"):
        super().__init__(width, height)
        self.output_file = output_file
        self.entity_1_num = num_1
        self.entity_2_num = num_2
        self.robot_num = robot_num
        self.init_entities()

    def init_entities(self):
        range_a = Landmark(landmark_id=self.entity_1_num + self.entity_2_num + self.robot_num,
                           initial_position=(50, 500),
                           size=np.array((100, 1000)),
                           color=(255, 182, 193))
        range_b = Landmark(landmark_id=self.entity_1_num + self.entity_2_num + self.robot_num + 1,
                           initial_position=(950, 500),
                           size=np.array((100, 1000)),
                           color=(255, 255, 224))

        self.add_entity(range_a)
        self.add_entity(range_b)
        robot_points = self.sample_points_inside_circle(350, (500, 500),
                                                        self.entity_1_num + self.entity_2_num + self.robot_num,
                                                        50)
        for entity_id, initial_position in enumerate(robot_points[:self.robot_num]):
            robot = Robot(robot_id=entity_id,
                          initial_position=initial_position,
                          size=10.0)
            self.add_entity(robot)
        for entity_id, initial_position in enumerate(robot_points[self.robot_num:self.entity_1_num + self.robot_num],
                                                     start=self.robot_num):
            object = PushableObject(object_id=entity_id,
                                    initial_position=initial_position,
                                    size=10.0,
                                    color='red')

            self.add_entity(object)
        for entity_id, initial_position in enumerate(robot_points[self.entity_1_num + self.robot_num:],
                                                     start=self.entity_1_num + self.robot_num):
            object = PushableObject(object_id=entity_id,
                                    initial_position=initial_position,
                                    size=10.0,
                                    color=(255, 200, 40))
            self.add_entity(object)

    # def update(self, dt: float):
    #     for entity in self.entities:
    #         if entity.__class__.__name__ == "Prey":
    #             entity.avoid_robots_and_walls(self.get_entities_by_type(Robot))
    #         entity.move(dt)

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

import json
import time

import numpy as np
import pygame

from modules.deployment.entity.base_entity import Entity
from modules.deployment.entity import Landmark, PushableObject, Robot
from modules.deployment.env import EnvironmentBase


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
                           color='green')
        range_b = Landmark(landmark_id=self.entity_1_num + self.entity_2_num + self.robot_num + 1,
                           initial_position=(950, 500),
                           size=np.array((100, 1000)),
                           color='red')

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
            object.density = 0.01

            self.add_entity(object)
        for entity_id, initial_position in enumerate(robot_points[self.entity_1_num + self.robot_num:],
                                                     start=self.entity_1_num + self.robot_num):
            object = PushableObject(object_id=entity_id,
                                    initial_position=initial_position,
                                    size=10.0,
                                    color='yellow')
            object.density = 0.01
            self.add_entity(object)

    def update(self, dt: float):
        if self._dt is None:
            self._dt = dt
        start_time = time.time()

        # Apply physics to predict new positions and handle collisions
        state = self._engine.step(self._dt)
        entities = self.entities.copy()
        for entity in entities:
            if isinstance(entity, PushableObject):
                if entity.position[0] < 100 or entity.position[0] > 900:
                    self.remove_entity(entity.id)
            # if entity.moveable:
            #     entity.position = state[0][entity.id]
            #     entity.velocity = state[1][entity.id]

        end_time = time.time()
        update_duration = end_time - start_time
        print(f"Physics update took {update_duration:.6f} seconds")

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

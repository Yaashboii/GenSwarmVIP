import json

import numpy as np
import pygame

from modules.deployment.entity import Obstacle, PushableObject, Robot

from modules.deployment.env.base_env import EnvironmentBase
from modules.deployment.engine import Box2DEngine


class MoveEnvironment(EnvironmentBase):
    def __init__(self,
                 width: int,
                 height: int,
                 robot_num: int,
                 obstacle_num: int,
                 output_file: str = "output.json"):
        super().__init__(width, height)
        self.output_file = output_file
        self.robot_num = robot_num
        self.obstacle_num = obstacle_num
        # self.engine = Box2DEngine()

        self.init_entities()

    def init_entities(self):
        robot_points = self.sample_points_inside_circle(400, (500, 500),
                                                        self.robot_num,
                                                        50)

        for entity_id, initial_position in enumerate(robot_points):
            robot = Robot(robot_id=entity_id,
                          initial_position=initial_position,
                          target_position=None,
                          size=10.0)
            self.add_entity(robot)
        # obstacle_points = [(200, 800), (350, 600), (500, 400), (650, 600), (800, 800)]
        # for entity_id, initial_position in enumerate(obstacle_points, start=len(robot_points)):
        #     obstacle = Obstacle(obstacle_id=entity_id,
        #                         initial_position=initial_position,
        #                         size=90.0)
        #     self.add_entity(obstacle)
        object = PushableObject(object_id=self.robot_num,
                                initial_position=(400, 600),
                                size=30.0,
                                color='red')
        object.density = 0.5
        object.target_position = (700, 200)
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

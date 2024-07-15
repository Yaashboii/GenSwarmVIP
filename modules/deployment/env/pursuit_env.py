import json
import time

import numpy as np
import pygame

from modules.deployment.entity import Obstacle, Prey, Robot

from modules.deployment.env.base_env import EnvironmentBase


class PursuitEnvironment(EnvironmentBase):
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
        self.init_entities()

    def init_entities(self):
        robot_points = self.sample_points_inside_circle(450, (500, 500),
                                                        self.robot_num + self.obstacle_num,
                                                        50)
        for entity_id, initial_position in enumerate(robot_points[:self.robot_num]):
            robot = Robot(robot_id=entity_id,
                          initial_position=initial_position,
                          target_position=None,
                          size=10.0)
            self.add_entity(robot)
        for entity_id, initial_position in enumerate(robot_points[self.robot_num:], start=self.robot_num):
            obstacle = Obstacle(obstacle_id=entity_id,
                                initial_position=initial_position,
                                size=23.0)
            self.add_entity(obstacle)

        prey = Prey(prey_id=self.robot_num + self.obstacle_num,
                    initial_position=(500, 500),
                    size=10.0,
                    )
        self.add_entity(prey)

    def update(self, dt: float):
        if self.dt is None:
            self.dt = dt
        start_time = time.time()

        # Apply physics to predict new positions and handle collisions
        self.engine.step(self.dt)
        state = self.engine.get_all_bodies_state()
        for entity in self.entities:
            if entity.__class__.__name__ == "Prey":
                entity.avoid_robots_and_walls(self.get_entities_by_type('Robot'))
                self.set_entity_velocity(entity.id, entity.velocity)
            if entity.moveable:
                entity.position, entity.velocity = state[entity.id]

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

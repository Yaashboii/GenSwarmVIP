import json

import numpy as np
import pygame

from modules.env.entity import Leader, Obstacle, Landmark, PushableObject, Robot
from modules.env.env.env import EnvironmentBase


class CrossEnvironment(EnvironmentBase):
    def __init__(self,
                 width: int,
                 height: int,
                 radius: int,
                 robot_num: int,
                 obstacle_num: int,
                 center: tuple = (500, 500),
                 output_file: str = "output.json"):
        super().__init__(width, height)
        self.radius = radius
        self.center = center
        self.output_file = output_file
        self.robot_num = robot_num
        self.obstacle_num = obstacle_num
        self.init_entities()

    def init_entities(self):
        robot_points = self.sample_points_on_circle(self.radius, self.center, self.robot_num)
        farthest_points = self.find_farthest_points(robot_points)
        for entity_id, (initial_position, target_position) in enumerate(zip(robot_points, farthest_points)):
            robot = Robot(robot_id=entity_id,
                          initial_position=initial_position,
                          target_position=target_position,
                          size=7.0)
            self.add_entity(robot)

        obstacle_points = self.sample_points_inside_circle(self.radius, self.center, self.obstacle_num, 50)
        for entity_id, initial_position in enumerate(obstacle_points, start=len(robot_points)):
            obstacle = Obstacle(obstacle_id=entity_id,
                                initial_position=initial_position,
                                size=20.0)
            self.add_entity(obstacle)

    @staticmethod
    def find_farthest_points(points):
        points = np.array(points)
        distances = np.linalg.norm(points[:, np.newaxis] - points, axis=2)
        farthest_indices = np.argmax(distances, axis=1)
        return points[farthest_indices]

    @staticmethod
    def sample_points_on_circle(radius, center, num_points):
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        points = [(center[0] + radius * np.cos(angle),
                   center[1] + radius * np.sin(angle)) for angle in angles]
        return points

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


if __name__ == '__main__':
    env = CrossEnvironment(width=1000, height=1000, radius=400, robot_num=6, obstacle_num=10)
    env.init_entities()
    env.run()
    print("Simulation completed successfully.")

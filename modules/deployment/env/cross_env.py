import json

import numpy as np
import pygame

from modules.deployment.entity import Leader, Obstacle, Robot
from modules.deployment.env import EnvironmentBase


class CrossEnvironment(EnvironmentBase):
    def __init__(self,
                 radius: int,
                 robot_num: int,
                 obstacle_num: int,
                 data_file: str = None,
                 center: tuple = (500, 500),
                 output_file: str = "output.json"):
        super().__init__(data_file=data_file, engine_type='OmniEngine')
        super().__init__(data_file=data_file, engine_type='QuadTreeEngine')
        self.radius = radius
        self.center = center
        self.output_file = output_file
        self.robot_num = robot_num
        self.obstacle_num = obstacle_num
        self.init_entities()

    def init_entities(self):

        obstacle_points = self.sample_points(self.center, self.obstacle_num, size=self.radius,
                                             min_distance=0.3)
        robot_points = self.sample_points_on_circle(self.radius, self.center, self.robot_num)
        farthest_points = self.find_farthest_points(robot_points)
        for entity_id, initial_position in enumerate(obstacle_points, start=len(robot_points)):
            obstacle = Obstacle(obstacle_id=entity_id,
                                initial_position=initial_position,
                                size=0.15)
            self.add_entity(obstacle)

        for entity_id, (initial_position, target_position) in enumerate(zip(robot_points, farthest_points)):
            robot = Robot(robot_id=entity_id+self.obstacle_num,
                          initial_position=initial_position,
                          target_position=target_position,
                          size=0.15)
            self.add_entity(robot)

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


if __name__ == '__main__':
    env = CrossEnvironment(width=1000, height=1000, radius=400, robot_num=6, obstacle_num=10)
    env.init_entities()
    env.run()
    print("Simulation completed successfully.")

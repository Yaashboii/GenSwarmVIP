import json
import time

import numpy as np
import pygame

from modules.deployment.entity import Robot, Landmark
from modules.deployment.env import EnvironmentBase


class ExploreEnvironment(EnvironmentBase):
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
        entity_id = 0
        for x in range(50, 1001, 100):
            for y in range(50, 1001, 100):
                landmark = Landmark(landmark_id=self.robot_num + entity_id,
                                    initial_position=(x, y),
                                    size=np.array([100, 100]),
                                    color='gray')
                self.add_entity(landmark)
                entity_id += 1
        robot_points = self.sample_points(center=(500, 500),
                                          num_points=self.robot_num,
                                          min_distance=20,
                                          shape='rectangle',
                                          size=(100, 100))
        for entity_id, initial_position in enumerate(robot_points):
            robot = Robot(robot_id=entity_id,
                          initial_position=initial_position,
                          target_position=None,
                          size=10.0)
            self.add_entity(robot)

    def update(self, dt: float):
        if self.dt is None:
            self.dt = dt
        start_time = time.time()

        # Apply physics to predict new positions and handle collisions
        self.engine.step(self.dt)
        state = self.engine.get_all_bodies_state()
        for entity in self.entities:
            if isinstance(entity, Robot):
                for landmark in self.entities:
                    if isinstance(landmark, Landmark):
                        if self.is_robot_within_landmark(entity, landmark):
                            landmark.color = 'blue'
            if entity.moveable:
                entity.position, entity.velocity = state[entity.id]

        end_time = time.time()
        update_duration = end_time - start_time
        print(f"Physics update took {update_duration:.6f} seconds")

    # def update_landmark_colors(self):
    #     if isinstance(entity, Robot):
    #         for landmark in self.entities:
    #             if isinstance(landmark, Landmark):
    #                 if self.is_robot_within_landmark(entity, landmark):
    #                     landmark.color = 'red'

    def is_robot_within_landmark(self, robot: Robot, landmark: Landmark):
        distance = np.linalg.norm(robot.position - landmark.position)
        landmark_radius = np.linalg.norm(landmark.size) / 2  # Assuming size is the diameter
        return distance <= landmark_radius

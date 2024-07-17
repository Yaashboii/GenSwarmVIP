import json

import numpy as np
import pygame

from modules.deployment.entity import Landmark, Leader, Obstacle, PushableObject, Robot
from modules.deployment.env import EnvironmentBase


class ConfigurableEnvironment(EnvironmentBase):
    def __init__(self, data_file: str = None, output_file: str = "output.json"):
        super().__init__(data_file)
        self.output_file = output_file
        self.generated_entities = []
        if self.data_file:
            self.add_entities_from_config()

    def add_entities_from_config(self):

        def add_specified_entities(entity_type, entity_class, color=None):
            nonlocal entity_id
            for entity_data in self.data["entities"][entity_type]["specified"]:
                entity = entity_class(entity_id, entity_data["position"], entity_data["size"])
                if color:
                    entity.color = entity_data.get("color", color)
                self.add_entity(entity)
                self.generated_entities.append(entity)
                entity_id += 1

        entity_id = 0

        add_specified_entities("leader", Leader, "red")
        add_specified_entities("obstacle", Obstacle)
        add_specified_entities("landmark", Landmark)
        add_specified_entities("pushable_object", PushableObject)
        add_specified_entities("robot", Robot, "green")

        # Add remaining robots
        if "count" in self.data["entities"]["robot"]:
            size = self.data["entities"]["robot"]["size"]
            robot_num = self.data["entities"]["robot"]["count"] - len(self.data["entities"]["robot"]["specified"])
            shape = self.data["entities"]["robot"]["shape"]
            color = self.data["entities"]["robot"]["color"]
            position = self.sample_points(center=[0, 0], num_points=robot_num, min_distance=size * 2, shape='rectangle',
                                          size=[self.width, self.height])
            for i in range(robot_num):
                robot = Robot(entity_id, position[i], size, color=color)
                self.add_entity(robot)
                self.generated_entities.append(robot)
                entity_id += 1


if __name__ == '__main__':
    env = ConfigurableEnvironment("../../../config/env_config.json")

    screen = pygame.display.set_mode((env.width, env.height))
    clock = pygame.time.Clock()

    import time

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            dt = clock.tick(10) / 1000
            env.update(dt)
            env.draw(screen)
            frame = pygame.surfarray.array3d(screen).astype(np.uint8)
            frame = np.rot90(frame, 3)
            frame = np.flip(frame, axis=1)

            time.sleep(0.1)
    finally:
        print("Shutting down")
        pygame.quit()
        env.save_entities_to_file()

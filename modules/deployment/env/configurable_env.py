import json

import numpy as np
import pygame

from modules.deployment.entity.entity import Entity
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
        def generate_random_position(entity_size, entity_shape, max_attempts=1000):
            attempts = 0
            while attempts < max_attempts:
                position = np.random.uniform([entity_size, entity_size],
                                             [self.width - entity_size, self.height - entity_size])
                if not any(
                        self._check_collision(position, entity_size, entity_shape, other) for other in self.entities):
                    return position
                attempts += 1
            raise ValueError(f"Failed to generate non-colliding position after {max_attempts} attempts.")

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
        for _ in range(self.data["entities"]["robot"]["count"] - len(self.data["entities"]["robot"]["specified"])):
            size = self.data["entities"]["robot"]["size"]
            shape = self.data["entities"]["robot"]["shape"]
            color = self.data["entities"]["robot"]["color"]
            position = generate_random_position(size, shape)
            robot = Robot(entity_id, position, size, color=color)
            self.add_entity(robot)
            self.generated_entities.append(robot)
            entity_id += 1

    def _check_collision(self, position, size, shape, other):
        if shape == 'circle':
            if other.shape == 'circle':
                return np.linalg.norm(position - other.position) < (size + other.size)
            else:
                return self.check_circle_rectangle_collision(other, position, size)
        else:
            if other.shape == 'circle':
                return self.check_circle_rectangle_collision(other, position, size)
            else:
                rect1 = pygame.Rect(position[0] - size[0] / 2, position[1] - size[1] / 2, size[0], size[1])
                rect2 = pygame.Rect(other.position[0] - other.size[0] / 2, other.position[1] - other.size[1] / 2,
                                    other.size[0], other.size[1])
                return rect1.colliderect(rect2)

    def check_circle_rectangle_collision(self, rect, circle_position, circle_radius):
        circle_center = circle_position
        rect_center = rect.position
        rect_half_size = np.array(rect.size) / 2

        # Find the closest point to the circle within the rectangle
        closest_point = np.clip(circle_center, rect_center - rect_half_size, rect_center + rect_half_size)

        # Calculate the distance between the circle's center and this closest point
        distance = np.linalg.norm(circle_center - closest_point)

        return distance < circle_radius

    def save_entities_to_file(self):
        entities_data = []
        for entity in self.generated_entities:
            entity_data = {
                'type': entity.__class__.__name__,
                "id": entity.id,
                "position": entity.position.tolist(),
                "size": entity.size if isinstance(entity.size, float) else entity.size.tolist(),
                "color": entity.color,
                "moveable": entity.moveable,
                "collision": entity.collision
            }
            entities_data.append(entity_data)

        with open(self.output_file, 'w') as file:
            json.dump(entities_data, file, indent=4)

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

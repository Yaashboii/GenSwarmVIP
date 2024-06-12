import json

import pygame

from modules.env.entity import Obstacle


class EnvironmentBase:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.entities = []

    def add_entity(self, entity):
        self.entities.append(entity)

    def get_entities_by_type(self, entity_type):
        """Get a list of entities of a specified type."""
        return [entity for entity in self.entities if isinstance(entity, entity_type)]

    def get_entity_position(self, entity_id):
        """Get the position of the entity with the specified ID."""
        for entity in self.entities:
            if entity.id == entity_id:
                return entity.position
        raise ValueError(f"No entity with ID {entity_id} found.")

    def get_entity_velocity(self, entity_id):
        """Get the velocity of the entity with the specified ID."""
        for entity in self.entities:
            if entity.id == entity_id:
                return entity.velocity
        raise ValueError(f"No entity with ID {entity_id} found.")

    def set_entity_position(self, entity_id, new_position):
        """Set the position of the entity with the specified ID."""
        for entity in self.entities:
            if entity.id == entity_id:
                entity.position = new_position
                return
        raise ValueError(f"No entity with ID {entity_id} found.")

    def set_entity_velocity(self, entity_id, new_velocity):
        """Set the velocity of the entity with the specified ID."""
        for entity in self.entities:
            if entity.id == entity_id:
                entity.velocity = new_velocity
                return
        raise ValueError(f"No entity with ID {entity_id} found.")

    def _get_entity_by_id(self, entity_id):
        for entity in self.entities:
            if entity.id == entity_id:
                return entity
        raise ValueError(f"No entity with ID {entity_id} found.")

    def connect_entities(self, entity1_id, entity2_id):
        entity1 = self._get_entity_by_id(entity1_id)
        entity2 = self._get_entity_by_id(entity2_id)
        entity1.connected_to(entity2)

    def disconnect_entities(self, entity1_id, entity2_id):
        entity1 = self._get_entity_by_id(entity1_id)
        entity2 = self._get_entity_by_id(entity2_id)
        entity1.disconnect_from(entity2)

    def add_walls(self, wall_thickness=10):
        walls = [
            Obstacle(obstacle_id=1, initial_position=[self.width / 2, wall_thickness / 2],
                     size=[self.width, wall_thickness]),
            Obstacle(obstacle_id=2, initial_position=[self.width / 2, self.height - wall_thickness / 2],
                     size=[self.width, wall_thickness]),
            Obstacle(obstacle_id=3, initial_position=[wall_thickness / 2, self.height / 2],
                     size=[wall_thickness, self.height]),
            Obstacle(obstacle_id=4, initial_position=[self.width - wall_thickness / 2, self.height / 2],
                     size=[wall_thickness, self.height])
        ]
        for wall in walls:
            self.add_entity(wall)

    def update(self, dt: float):
        for entity in self.entities:
            entity.move(dt)

    def get_observation(self):

        obs = {}
        for entity in self.entities:
            obs[entity.id] = {
                "position": entity.position,
                "velocity": entity.velocity,
                "size": entity.size,
                "type": entity.__class__.__name__
            }
        return obs

    def draw(self, screen):
        screen.fill((255, 255, 255))
        for entity in self.entities:
            if entity.shape == 'circle':
                pygame.draw.circle(screen, pygame.Color(entity.color), entity.position.astype(int), int(entity.size))
            else:
                rect = pygame.Rect(entity.position[0] - entity.size[0] / 2, entity.position[1] - entity.size[1] / 2,
                                   entity.size[0], entity.size[1])
                pygame.draw.rect(screen, pygame.Color(entity.color), rect)
        pygame.display.flip()

    def save_entities_to_file(self):
        entities_data = []
        for entity in self.entities:
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

import json
import time
import numpy as np
import pygame
from modules.env.entity import Obstacle
from multiprocessing import Pool

class EnvironmentBase:
    def __init__(self, width: int, height: int, use_quad_tree: bool = True):
        self.width = width
        self.height = height
        self.entities = []
        self.use_quad_tree = use_quad_tree  # Define the use of quad-tree for collision detection
        self.output_file = "entities.json"  # Define the output file for saving entities

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
        start_time = time.time()

        # Apply physics to predict new positions and handle collisions
        self.apply_physics_and_handle_collisions(dt)

        end_time = time.time()
        update_duration = end_time - start_time
        print(f"Physics update took {update_duration:.6f} seconds")

    def apply_physics_and_handle_collisions(self, dt: float, friction: float = 0.01):
        positions = np.array([entity.position for entity in self.entities if entity.collision])
        velocities = np.array([entity.velocity for entity in self.entities if entity.collision])
        forces = np.array([entity.force for entity in self.entities if entity.collision])
        masses = np.array([entity.mass for entity in self.entities if entity.collision])

        # Calculate initial accelerations
        accelerations = forces / masses[:, np.newaxis]

        # Apply friction
        friction_forces = -friction * velocities
        total_forces = forces + friction_forces

        # Update accelerations with friction
        accelerations = total_forces / masses[:, np.newaxis]

        # Predict new positions
        predicted_positions = positions + velocities * dt + 0.5 * accelerations * dt ** 2

        # Handle collisions based on predicted positions
        self.handle_collisions(predicted_positions)

        # Recalculate forces based on collisions
        forces = np.array([entity.force for entity in self.entities if entity.collision])
        total_forces = forces + friction_forces

        # Recalculate accelerations with new forces
        accelerations = total_forces / masses[:, np.newaxis]

        # Update velocities
        velocities += accelerations * dt
        velocities = np.clip(velocities, -50, 50)

        # Update positions with new velocities and accelerations
        positions += velocities * dt + 0.5 * accelerations * dt ** 2

        # Update entities with new positions and velocities
        movable_entities = [entity for entity in self.entities if entity.collision]
        for i, entity in enumerate(movable_entities):
            entity.position = positions[i]
            entity.velocity = velocities[i]

    def handle_collisions(self, predicted_positions):
        if self.use_quad_tree:
            self._handle_collisions_quad_tree(predicted_positions)
        else:
            self._handle_collisions_iteration(predicted_positions)

    def _handle_collisions_quad_tree(self, predicted_positions):
        quad_tree = QuadTree(0, 0, self.width, self.height)
        for i, entity in enumerate(self.entities):
            if entity.collision:
                entity.predicted_position = predicted_positions[i]
                quad_tree.insert(entity)

        for entity in self.entities:
            if entity.collision:
                possible_collisions = quad_tree.retrieve(entity)
                for other in possible_collisions:
                    if entity != other and self.check_collision(entity, other, entity.predicted_position,
                                                                other.predicted_position):
                        self.resolve_collision(entity, other)

    def _handle_collisions_iteration(self, predicted_positions):
        for i, entity1 in enumerate(self.entities):
            if entity1.collision:
                entity1.predicted_position = predicted_positions[i]
                for j, entity2 in enumerate(self.entities):
                    if i != j and entity2.collision:
                        entity2.predicted_position = predicted_positions[j]
                        if self.check_collision(entity1, entity2, entity1.predicted_position,
                                                entity2.predicted_position):
                            self.resolve_collision(entity1, entity2)

    def check_collision(self, entity1, entity2, position1, position2):
        distance = np.linalg.norm(position1 - position2)
        return distance <= (entity1.size + entity2.size)

    def resolve_collision(self, entity1, entity2):
        normal = entity1.predicted_position - entity2.predicted_position
        distance = np.linalg.norm(normal)

        if distance == 0:
            normal = np.random.randn(2)
            normal /= np.linalg.norm(normal)
            distance = 1e-6
        else:
            normal /= distance

        penetration_depth = entity1.size + entity2.size - distance

        if penetration_depth > 0:
            relative_velocity = entity1.velocity - entity2.velocity
            velocity_along_normal = np.dot(relative_velocity, normal)

            if velocity_along_normal > 0:
                return

            restitution = min(entity1.restitution, entity2.restitution)

            impulse_magnitude = -(1 + restitution) * velocity_along_normal
            impulse_magnitude /= (1 / entity1.mass + 1 / entity2.mass)

            k = 10  # 碰撞刚度系数
            penetration_force_magnitude = k * penetration_depth

            impulse_magnitude = min(impulse_magnitude, penetration_force_magnitude)

            impulse = impulse_magnitude * normal
            penetration_force = penetration_force_magnitude * normal

            entity1.force += (impulse + penetration_force) / entity1.mass
            entity2.force -= (impulse + penetration_force) / entity2.mass

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

    def generate_matrices(self):
        sorted_entities = sorted(self.entities, key=lambda x: x.id)

        positions = []
        velocities = []
        accelerations = []
        masses = []

        for entity in sorted_entities:
            if entity.collision:
                positions.append(entity.position)
                velocities.append(entity.velocity)
                accelerations.append(entity.acceleration)
                masses.append(entity.mass)

        position_matrix = np.array(positions)
        velocity_matrix = np.array(velocities)
        acceleration_matrix = np.array(accelerations)
        mass_matrix = np.array(masses)

        return position_matrix, velocity_matrix, acceleration_matrix, mass_matrix


class QuadTree:
    MAX_ENTITIES = 4  # Increased to 8 for better performance
    MAX_LEVELS = 5

    def __init__(self, x, y, width, height, level=0):
        self.bounds = pygame.Rect(x, y, width, height)
        self.level = level
        self.entities = []
        self.nodes = []

    def insert(self, entity):
        if self.nodes:
            index = self.get_index(entity)
            if index != -1:
                self.nodes[index].insert(entity)
                return

        self.entities.append(entity)

        if len(self.entities) > self.MAX_ENTITIES and self.level < self.MAX_LEVELS:
            if not self.nodes:
                self.split()

            i = 0
            while i < len(self.entities):
                index = self.get_index(self.entities[i])
                if index != -1:
                    self.nodes[index].insert(self.entities.pop(i))
                else:
                    i += 1

    def split(self):
        sub_width = self.bounds.width / 2
        sub_height = self.bounds.height / 2
        x = self.bounds.x
        y = self.bounds.y

        self.nodes.append(QuadTree(x, y, sub_width, sub_height, self.level + 1))
        self.nodes.append(QuadTree(x + sub_width, y, sub_width, sub_height, self.level + 1))
        self.nodes.append(QuadTree(x, y + sub_height, sub_width, sub_height, self.level + 1))
        self.nodes.append(QuadTree(x + sub_width, y + sub_height, sub_width, sub_height, self.level + 1))

    def get_index(self, entity):
        index = -1
        vertical_midpoint = self.bounds.x + (self.bounds.width / 2)
        horizontal_midpoint = self.bounds.y + (self.bounds.height / 2)

        top_quadrant = entity.position[1] < horizontal_midpoint and entity.position[
            1] + entity.size / 2 < horizontal_midpoint
        bottom_quadrant = entity.position[1] > horizontal_midpoint

        if entity.position[0] < vertical_midpoint and entity.position[0] + entity.size / 2 < vertical_midpoint:
            if top_quadrant:
                index = 0
            elif bottom_quadrant:
                index = 2
        elif entity.position[0] > vertical_midpoint:
            if top_quadrant:
                index = 1
            elif bottom_quadrant:
                index = 3

        return index

    def retrieve(self, entity):
        index = self.get_index(entity)
        return_entities = self.entities.copy()

        if self.nodes:
            if index != -1:
                return_entities.extend(self.nodes[index].retrieve(entity))
            else:
                for node in self.nodes:
                    return_entities.extend(node.retrieve(entity))

        return return_entities

    def clear(self):
        self.entities = []
        for node in self.nodes:
            node.clear()
        self.nodes = []

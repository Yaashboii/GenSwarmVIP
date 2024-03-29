import numpy as np


class Entity:
    def __init__(self, entity_id, initial_position, radius):
        self._id = entity_id
        self._position = np.array(initial_position, dtype=float)
        self._radius = radius

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = np.array(value, dtype=float)

    @property
    def radius(self):
        return self._radius

    @property
    def id(self):
        return self._id

    @classmethod
    def is_overlapping(cls, new_entity, entities):
        for entity in entities:
            if np.linalg.norm(new_entity.position - entity.position) < (new_entity.radius + entity.radius):
                return True
        return False

    @classmethod
    def try_generate_entity(cls, index: int, size: tuple, radius: float, entities: list, max_attempts=100):
        for attempt in range(max_attempts):
            position = np.random.uniform(-0.5, 0.5, size=2) * size
            new_entity = cls(index, position, radius)
            if not cls.is_overlapping(new_entity, entities):
                return new_entity
        return None

    @classmethod
    def create_entities(cls, n_entities: int, size: tuple, radius_range: tuple | float, existing_entities: list = []):
        entities = existing_entities.copy()
        created_entities = []
        for i in range(n_entities):
            if isinstance(radius_range, tuple):
                radius = np.random.uniform(*radius_range)
            else:
                radius = radius_range
            new_entity = cls.try_generate_entity(i, size, radius, entities + created_entities)
            if new_entity:
                created_entities.append(new_entity)
            else:
                print(f"Warning: Failed to place entity {i} after multiple attempts.")
        return created_entities


class Obstacle(Entity):
    def __init__(self, obstacle_id, initial_position, radius):
        super().__init__(obstacle_id, initial_position, radius)


class Obstacles:
    def __init__(self, n_obstacles: int, size: list, robot_list: list):
        self._obstacles = self.create_obstacles(n_obstacles, size, robot_list)

    @property
    def obstacles(self):
        return self._obstacles

    @staticmethod
    def create_obstacles(n_obstacles, size, robot_list: list):
        # TODO: optimize this function to avoid obstacles overlapping
        obstacle_list = []
        while len(obstacle_list) < n_obstacles:
            new_obstacle = Obstacle.create_entities(n_entities=n_obstacles,
                                                    size=size,
                                                    radius_range=(0.5, 1.0),
                                                    existing_entities=obstacle_list + robot_list)
            if new_obstacle:
                obstacle_list.extend(new_obstacle)
            else:
                print(f"Warning: Failed to place obstacle {len(obstacle_list)} after multiple attempts.")

        return obstacle_list

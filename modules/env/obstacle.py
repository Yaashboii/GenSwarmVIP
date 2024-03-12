import numpy as np


class Entity:
    def __init__(self, entity_id, initial_position):
        self._id = entity_id
        self._position = np.array(initial_position, dtype=float)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = np.array(value, dtype=float)

    @property
    def id(self):
        return self._id


class Obstacle(Entity):
    def __init__(self, obstacle_id, initial_position, radius):
        super().__init__(obstacle_id, initial_position)
        self._radius = radius

    @property
    def radius(self):
        return self._radius


class Obstacles:
    def __init__(self, n_obstacles, size):
        self._obstacles = self.create_obstacles(n_obstacles, size)

    @property
    def obstacles(self):
        return self._obstacles

    @staticmethod
    def create_obstacles(n_obstacles, size):
        # TODO: optimize this function to avoid obstacles overlapping
        obstacle_list = []
        for i in range(n_obstacles):
            position = np.random.uniform(-0.5, 0.5, size=2) * size
            radius = np.random.uniform(0.2, 1.0)
            obstacle_list.append(Obstacle(i, position, radius=radius))

        return obstacle_list

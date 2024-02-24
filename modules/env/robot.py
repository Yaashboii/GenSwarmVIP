import numpy as np


class Robot:
    def __init__(self, robot_id, initial_position, max_speed=2.0):
        self._robot_id = robot_id
        self._position = np.array(initial_position, dtype=float)
        self._velocity = np.array([0.0, 0.0], dtype=float)
        self._max_speed = max_speed
        self._history = [self._position.copy()]

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        self._velocity = np.array(value, dtype=float)
        if np.linalg.norm(self.velocity) > self._max_speed:
            self._velocity = self.velocity / np.linalg.norm(self.velocity) * self._max_speed

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = np.array(value, dtype=float)

    @property
    def history(self):
        return self._history

    @history.setter
    def history(self, value):
        self._history = value

    @property
    def robot_id(self):
        return self._robot_id

    @property
    def max_speed(self):
        return self._max_speed


def create_robots(n_robots, initial_positions):
    robots_list = []
    for i in range(n_robots):
        robots_list.append(Robot(i, initial_positions[i]))
    return robots_list


def generate_robots_random_positions(n_robots, size):
    return [[np.random.uniform(0, size[0]), np.random.uniform(0, size[1])] for _ in range(n_robots)]


robots = create_robots(6, generate_robots_random_positions(6, [10, 10]))
pass
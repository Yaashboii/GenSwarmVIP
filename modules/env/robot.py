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
    def velocity(self, new_velocity):
        if np.linalg.norm(new_velocity) > self._max_speed:
            new_velocity = new_velocity / np.linalg.norm(new_velocity) * self._max_speed
        self._velocity = np.array(new_velocity, dtype=float)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = np.array(value, dtype=float)
        self._history.append(self._position)

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


class Robots:
    def __init__(self, n_robots, env_size):
        self._env_size = env_size
        initial_positions = self.generate_robots_random_positions(n_robots)
        self._robots = self.create_robots(n_robots, initial_positions)
        self._positions = np.array([robot.position for robot in self._robots])
        self._velocities = np.array([robot.velocity for robot in self._robots])
        self._history = [self._positions]

    @staticmethod
    def create_robots(n_robots, initial_positions):
        robots_list = []
        for i in range(n_robots):
            # id starts from 1
            robots_list.append(Robot(i + 1, initial_positions[i]))
        return robots_list

    def generate_robots_random_positions(self, n_robots):
        return [[np.random.uniform(-self._env_size[0] / 2, self._env_size[0] / 2),
                 np.random.uniform(-self._env_size[1] / 2, self._env_size[1] / 2)] for _ in range(n_robots)]

    @property
    def positions(self):
        self._positions = np.array([robot.position for robot in self._robots])
        return self._positions

    @positions.setter
    def positions(self, new_positions):
        assert new_positions.shape == self._positions.shape, f"Expected shape {self._positions.shape}, got {new_positions.shape}"
        self._positions = new_positions
        for i, robot in enumerate(self._robots):
            robot.position = new_positions[i]
        self._history.append(self._positions)

    @property
    def velocities(self):
        self._velocities = np.array([robot.velocity for robot in self._robots])
        return self._velocities

    @velocities.setter
    def velocities(self, new_velocities):
        assert new_velocities.shape == self._velocities.shape, f"Expected shape {self._velocities.shape}, got {new_velocities.shape}"
        self._velocities = new_velocities
        for i, robot in enumerate(self._robots):
            robot.velocity = new_velocities[i]

    @property
    def history(self):
        return np.array(self._history)

    @history.setter
    def history(self, value):
        self._history = value

    def move_robots(self, dt):
        """
        Move the robots according to their velocities.
        Args:
            dt: time step

        Returns: None

        """
        new_positions = self.positions + self.velocities * dt
        self.positions = np.clip(new_positions, -np.array(self._env_size) / 2, np.array(self._env_size) / 2)


class Leader(Robot):
    def __init__(self, initial_position, max_speed=3.0):
        super().__init__(0, initial_position, max_speed)
        self.trajectory = []
        self.angle = 0
        self._init_position = initial_position

    def move_in_circle(self, radius, speed, dt):
        speed = np.clip(speed, 0, self.max_speed)
        omega = speed / radius

        self.angle += omega * dt

        # 计算新位置
        new_x = radius * np.cos(self.angle)
        new_y = radius * np.sin(self.angle)
        self.position = np.array([new_x, new_y]) + self._init_position

        self.trajectory.append(self.position)

    def move(self, speed, dt, shape='circle'):
        # TODO add more methods to move the leader in different patterns
        if shape == 'circle':
            self.move_in_circle(4, speed, dt)

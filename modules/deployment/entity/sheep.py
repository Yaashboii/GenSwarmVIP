from .prey import Prey
import numpy as np


class Sheep(Prey):
    def __init__(self,
                 prey_id,
                 initial_position,
                 size,
                 max_speed=1,
                 danger_zone=1,
                 damping=0.9,
                 random_factor=0.1,
                 alpha=0.1,
                 density=0.1,
                 mass=1.0):
        super().__init__(prey_id, initial_position, size, mass=mass, density=density)

        self.max_speed = max_speed
        self.danger_zone = danger_zone  # 当离开狗这个圆半径后，就不会被狗影响速度
        self.damping = damping
        self.random_factor = random_factor
        self.alpha = alpha
        self.velocity = np.zeros(2)
        self.filtered_velocity = np.zeros(2)

    def calculate_velocity(self, flock, robots, environment_bounds):
        random_movement = np.random.randn(2) * self.random_factor

        repulsion = self.separate(flock + robots)

        # 边缘避免行为
        avoid_edge = self.avoid_edges(environment_bounds=environment_bounds)
        avoid_dogs = self.avoid_dogs(flock=flock, robots=robots)
        new_velocity = (self.velocity +
                        2 * repulsion +
                        0 * avoid_edge +
                        2 * avoid_dogs +
                        random_movement)

        self.filtered_velocity = self.alpha * new_velocity + (1 - self.alpha) * self.filtered_velocity

        speed = np.linalg.norm(self.filtered_velocity)
        if speed > self.max_speed:
            self.filtered_velocity = self.filtered_velocity / speed * self.max_speed

        self.velocity = self.filtered_velocity
        return self.velocity

    def avoid_edges(self, environment_bounds):
        avoidance_force = np.zeros(2)
        x, y = self.position

        if x < environment_bounds[0] + 0.5:
            avoidance_force[1] = 0.5 * (0.5 / (x - environment_bounds[0] + 1e-5))  # 加速因子
            avoidance_force[0] = -avoidance_force[0]
        elif x > environment_bounds[1] - 0.5:
            avoidance_force[1] = -0.5 * (0.5 / (environment_bounds[1] - x + 1e-5))  # 加速因子
            avoidance_force[0] = -avoidance_force[0]

        if y < environment_bounds[2] + 0.5:
            avoidance_force[0] = 0.5 * (0.5 / (y - environment_bounds[2] + 1e-5))  # 加速因子
            avoidance_force[1] = -avoidance_force[1]

        elif y > environment_bounds[3] - 0.5:
            avoidance_force[0] = -0.5 * (0.5 / (environment_bounds[3] - y + 1e-5))  # 加速因子
            avoidance_force[1] = -avoidance_force[1]

        if np.any(avoidance_force) != 0:
            avoidance_force = avoidance_force / np.linalg.norm(avoidance_force)
        return avoidance_force

    def separate(self, others):
        if len(others) == 0:
            return np.zeros(2)
        separation_force = np.zeros(2)
        for other in others:
            distance = np.linalg.norm(self.position - other.position)
            if distance < 0.5:  # 0.5m 范围内的排斥力
                separation_force -= (other.position - self.position) / (distance + 1e-5)
        if any(separation_force) != 0:
            separation_force = separation_force / np.linalg.norm(separation_force)
        return separation_force

    def avoid_dogs(self, robots, flock):
        new_velocity = 0
        if any([np.linalg.norm(self.position - dog.position) < self.danger_zone for dog in robots]):
            flock_center = np.mean([sheep.position for sheep in flock], axis=0)
            direction_to_flock = (flock_center - self.position)
            distance_to_flock = np.linalg.norm(direction_to_flock)
            distance_to_dog = min([np.linalg.norm(self.position - dog.position) for dog in robots])

            direction_to_flock_normalized = direction_to_flock / distance_to_flock if distance_to_flock != 0 else np.zeros(
                2)
            speed_factor = distance_to_dog / (distance_to_flock + 1e-5)
            new_velocity = direction_to_flock_normalized * speed_factor * self.max_speed
        return new_velocity

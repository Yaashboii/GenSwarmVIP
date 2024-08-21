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

    def calculate_velocity(self, flock, robots):
        alignment = self.align(flock)
        cohesion = self.cohere(flock)
        separation = self.separate(flock)
        avoidance = self.avoid_dogs(robots)
        random_movement = np.random.randn(2) * self.random_factor

        new_velocity = (alignment * 4 + cohesion * 1.0 + separation * 0.5 + avoidance * 4 + random_movement)

        self.filtered_velocity = self.alpha * new_velocity + (1 - self.alpha) * self.filtered_velocity

        speed = np.linalg.norm(self.filtered_velocity)
        if speed > self.max_speed:
            self.filtered_velocity = self.filtered_velocity / speed * self.max_speed

        self.velocity = self.filtered_velocity
        return self.velocity

    def align(self, flock):
        if len(flock) == 0:
            return np.zeros(2)
        avg_velocity = np.mean([sheep.velocity for sheep in flock], axis=0)
        return (avg_velocity - self.velocity) * 0.5  # 调整对齐权重

    def cohere(self, flock):
        if len(flock) == 0:
            return np.zeros(2)
        avg_position = np.mean([sheep.position for sheep in flock], axis=0)
        return (avg_position - self.position) * 0.35  # 调整聚集权重

    def separate(self, flock):
        if len(flock) == 0:
            return np.zeros(2)
        separation_force = np.zeros(2)
        for sheep in flock:
            distance = np.linalg.norm(self.position - sheep.position)
            if distance < (self.size + sheep.size) + 20:  # 距离阈值
                separation_force -= (sheep.position - self.position)
        return separation_force * 3.5  # 调整分离权重

    def avoid_dogs(self, dogs):
        if len(dogs) == 0:
            return np.zeros(2)
        avoidance_force = np.zeros(2)
        for dog in dogs:
            distance = np.linalg.norm(self.position - dog.position)
            if distance < self.danger_zone:
                if distance == 0:
                    distance = 0.01  # 防止除以零
                avoidance_strength = 1 / distance * 100  # 距离越近，力量越大
                avoidance_force -= (dog.position - self.position) * avoidance_strength
        return avoidance_force  # 调整避开狗的权重

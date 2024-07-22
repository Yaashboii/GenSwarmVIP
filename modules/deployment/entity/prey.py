import numpy as np

from .base_entity import Entity


class Prey(Entity):
    def __init__(self, prey_id, initial_position, size, mass, density):
        super().__init__(prey_id,
                         initial_position,
                         size,
                         color='blue',
                         collision=True,
                         moveable=True,
                         max_speed=1.0,
                         mass=mass,
                         density=density)
        self.__wall_avoidance_weight = 1.0
        self.__robot_avoidance_weight = 1.0
        self.__smooth_factor = 0.1

    def calculate_avoidance_vector(self, all_robots: list[Entity], separation_distance: float = 200):
        avoidance_vector = np.zeros(2, dtype=float)

        # Avoid robots
        for robot in all_robots:
            if robot.id != self.id:
                distance_vector = self.position - robot.position
                distance = np.linalg.norm(distance_vector)
                if distance < separation_distance:
                    avoidance_vector += distance_vector / distance  # Normalize and add

        if np.linalg.norm(avoidance_vector) > 0:
            avoidance_vector = (avoidance_vector / np.linalg.norm(avoidance_vector))

        return avoidance_vector

    def calculate_wall_avoidance_vector(self):
        wall_avoidance_vector = np.zeros(2, dtype=float)

        # Define the thresholds
        lower_threshold = 100
        upper_threshold = 900
        lower_bound = 0
        upper_bound = 1000

        # Check x-axis
        if self.position[0] < lower_threshold:
            wall_avoidance_vector[0] = (lower_threshold - self.position[0]) / lower_threshold
        elif self.position[0] > upper_threshold:
            wall_avoidance_vector[0] = (upper_threshold - self.position[0]) / (upper_bound - upper_threshold)

        # Check y-axis
        if self.position[1] < lower_threshold:
            wall_avoidance_vector[1] = (lower_threshold - self.position[1]) / lower_threshold
        elif self.position[1] > upper_threshold:
            wall_avoidance_vector[1] = (upper_threshold - self.position[1]) / (upper_bound - upper_threshold)

        if np.linalg.norm(wall_avoidance_vector) > 0:
            wall_avoidance_vector = (wall_avoidance_vector / np.linalg.norm(
                wall_avoidance_vector)) * self.__wall_avoidance_weight  # Normalize and apply weight

        return wall_avoidance_vector

    def avoid_robots_and_walls(self, all_robots: list[Entity], separation_distance: float = 100):
        if not self.moveable:
            return

        robot_avoidance_vector = self.calculate_avoidance_vector(all_robots, separation_distance)
        wall_avoidance_vector = self.calculate_wall_avoidance_vector()

        total_avoidance_vector = robot_avoidance_vector * self.__robot_avoidance_weight + wall_avoidance_vector * self.__wall_avoidance_weight

        # Adjust the velocity with the total avoidance vector
        new_velocity = total_avoidance_vector

        # Normalize the velocity and apply max speed
        if np.linalg.norm(new_velocity) > 0:
            new_velocity = (new_velocity / np.linalg.norm(new_velocity)) * self.max_speed

        # Smooth the velocity change
        self.velocity = self.velocity * (1 - self.__smooth_factor) + new_velocity * self.__smooth_factor

        # Cap the velocity to the maximum speed
        if np.linalg.norm(self.velocity) > self.max_speed:
            self.velocity = (self.velocity / np.linalg.norm(self.velocity)) * self.max_speed

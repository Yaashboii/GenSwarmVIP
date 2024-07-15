import time

import numpy as np

from modules.deployment.entity import Robot, Sheep

from modules.deployment.env.base_env import EnvironmentBase


class SheepdogEnvironment(EnvironmentBase):
    def __init__(self,
                 width: int,
                 height: int,
                 sheep_num: int,
                 dog_num: int,
                 output_file: str = "output.json"):
        super().__init__(width, height)
        self.landmark = None
        self.output_file = output_file
        self.sheep_num = sheep_num
        self.dog_num = dog_num
        self.init_entities()

    def init_entities(self):
        # wall_width = 10
        # wall = Wall(wall_id=0,
        #             initial_position=(0.5 * self._width, 0.5 * wall_width),
        #             size=(self._width, wall_width))
        # self.add_entity(wall)
        # wall = Wall(wall_id=1,
        #             initial_position=(0.5 * self._width, self._height - 0.5 * wall_width),
        #             size=(self._width, wall_width))
        # self.add_entity(wall)
        # wall = Wall(wall_id=2,
        #             initial_position=(0.5 * wall_width, 0.5 * self._height),
        #             size=(wall_width, self._height))
        # self.add_entity(wall)
        # wall = Wall(wall_id=3,
        #             initial_position=(self._width - 0.5 * wall_width, 0.5 * self._height),
        #             size=(wall_width, self._height))
        # self.add_entity(wall)
        # left_wall = Wall(wall_id=4,
        #                  initial_position=(200, 400),
        #                  size=(0.45 * self._width, wall_width))
        # self.add_entity(left_wall)
        # right_wall = Wall(wall_id=5,
        #                   initial_position=(800, 400),
        #                   size=(0.45 * self._width, wall_width))
        # self.add_entity(right_wall)

        start_num = len(self._entities)
        robot_points = self.sample_points(center=(500, 500),
                                          num_points=self.dog_num + self.sheep_num,
                                          min_distance=5,
                                          shape='rectangle',
                                          size=(1000, 1000),
                                          )
        size = 20.0
        for entity_id, initial_position in enumerate(robot_points[:self.dog_num]):
            # Calculate the angle for even distribution along the boundary
            angle = (entity_id / self.dog_num) * 2 * np.pi

            # Determine x and y coordinates based on angle
            if 0 <= angle < np.pi / 2:  # Top side
                x = 1000 * angle / (np.pi / 2)
                y = 0
            elif np.pi / 2 <= angle < np.pi:  # Right side
                x = 1000
                y = 1000 * (angle - np.pi / 2) / (np.pi / 2)
            elif np.pi <= angle < 3 * np.pi / 2:  # Bottom side
                x = 1000 * (1 - (angle - np.pi) / (np.pi / 2))
                y = 1000
            else:  # Left side
                x = 0
                y = 1000 * (1 - (angle - 3 * np.pi / 2) / (np.pi / 2))

            dog = Robot(robot_id=entity_id + start_num,
                        initial_position=np.array([x, y]),
                        size=size)
            self.add_entity(dog)
        for entity_id, initial_position in enumerate(robot_points[self.dog_num:], start=self.dog_num):
            sheep = Sheep(prey_id=entity_id + start_num,
                          initial_position=initial_position,
                          size=10.0)
            self.add_entity(sheep)

    def update(self, dt: float):
        if self._dt is None:
            self._dt = dt
        start_time = time.time()

        # Apply physics to predict new positions and handle collisions
        state = self._engine.step(self._dt)
        for entity in self._entities:
            if isinstance(entity, Sheep):
                # 获取邻居羊群和机器人列表
                flock = [e for e in self._entities if isinstance(e, Sheep) and e != entity]
                robots = [e for e in self._entities if isinstance(e, Robot)]
                speed = entity.calculate_velocity(flock, robots)
                self.set_entity_velocity(entity.id, speed)

        end_time = time.time()
        update_duration = end_time - start_time
        print(f"Physics update took {update_duration:.6f} seconds")

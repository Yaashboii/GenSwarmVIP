import json

import gymnasium
from gymnasium import spaces
import numpy as np
import pygame
from gymnasium.utils import seeding
from gymnasium.spaces import Box

from modules.deployment.engine import Box2DEngine, QuadTreeEngine, OmniEngine


class GymnasiumEnvironmentBase(gymnasium.Env):
    metadata = {
        'render.modes': ['human'],
        'fps': 30
    }

    def __init__(self, data_file: str):
        """
        Base class for environments.
        Args:
            data_file (str): The environment display info and entity info
        """
        self.data_file = data_file
        with open(self.data_file, 'r') as file:
            self.data = json.load(file)
        self._dt = None
        self._simulation_data = {}
        self._scale_factor = self.data['display']['scale_factor']
        self._width = self.data['display']['width']
        self._height = self.data['display']['height']

        self._entities = []
        engine_type = self.data['engine_type']
        if engine_type == 'QuadTreeEngine':
            self._engine = QuadTreeEngine(world_size=(self._width, self._height),
                                          alpha=0.9,
                                          damping=0.75,
                                          collision_check=True,
                                          joint_constraint=True)
        elif engine_type == 'Box2DEngine':
            self._engine = Box2DEngine()
        elif engine_type == 'Omni_Engine':
            self._engine = OmniEngine()
        else:
            raise ValueError(f"Unsupported engine type: {engine_type}")

        self.num_robots = self.data["entities"]["robot"]["count"] + self.data["entities"]["leader"]["count"]
        self.num_obstacles = self.data["entities"]["obstacle"]["count"]
        self.get_spaces()

        self.screen = pygame.display.set_mode((self.width * self.scale_factor, self.height * self.scale_factor))
        self.render_mode = self.data['render_mode']
        self.output_file = self.data['output_file']
        self.clock = pygame.time.Clock()
        self.FPS = 30

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def close(self):
        if self.screen is not None:
            pygame.quit()
            self.screen = None

    def convert_coordinates(self, value, option="position"):
        """This function converts coordinates in pymunk into pygame coordinates.

        The coordinate system in pygame is:
                 (0, 0) +-------+ (WINDOWSIZE, 0)           + ──── → x
                        |       |                           │
                        |       |                           │
        (0, WINDOWSIZE) +-------+ (WINDOWSIZE, WINDOWSIZE)  ↓ y
        The coordinate system in pymunk is:
        (0, WINDOWSIZE) +-------+ (WINDOWSIZE, WINDOWSIZE)  ↑ y
                        |       |                           │
                        |       |                           │
                 (0, 0) +-------+ (WINDOWSIZE, 0)           + ──── → x
        """
        if option == "position":
            return int(value[0]), - int(value[1])

        if option == "velocity":
            return value[0], -value[1]

    def get_spaces(self):
        """Define the action and observation spaces for all agents."""

        obs_dim = 3
        obs_space = spaces.Box(
            low=np.float32(-np.sqrt(2)),
            high=np.float32(np.sqrt(2)),
            shape=(obs_dim,),
            dtype=np.float32,
        )

        act_space = spaces.Box(
            low=np.float32(-1.0),
            high=np.float32(1.0),
            shape=(2,),
            dtype=np.float32,
        )

        self.observation_space = [obs_space for _ in range(self.num_robots)]
        self.action_space = [act_space for _ in range(self.num_robots)]

    def step(self):
        self._dt = self.clock.tick(self.FPS) / 1000
        self._engine.step(self._dt)
        self.render()
        obs = self.get_observation("array")
        reward = self.reward()
        termination = False
        truncation = False
        infos = self.get_observation("dict")
        return obs, reward, termination, truncation, infos

    def get_observation(self, type: str = "dict"):

        if type == "dict":
            obs = {}
            for entity in self._entities:
                obs[entity.id] = {
                    "position": entity.position,
                    "velocity": entity.velocity,
                    "size": entity.size,
                    "type": entity.__class__.__name__,
                    "target_position": entity.target_position if hasattr(entity, "target_position") else None,
                    "color": entity.color
                }
        elif type == "array":
            obs = []
            for entity in self._entities:
                obs.append(entity.position)

        return obs

    def reward(self):
        reward = {}
        for entity in self._entities:
            reward[entity.id] = 0
        return reward

    def render(self):
        if self.render_mode is None:
            gymnasium.logger.warn(
                "You are calling render method without specifying any render mode."
            )
            return

        self.draw()
        self.clock.tick(self.FPS)

        rgb_array = pygame.surfarray.pixels3d(self.screen)
        new_rgb_array = np.copy(rgb_array)
        del rgb_array

        if self.render_mode == "human":
            pygame.event.pump()
            pygame.display.update()
        return (
            np.transpose(new_rgb_array, axes=(1, 0, 2))
            if self.render_mode == "rgb_array"
            else None
        )

    def draw(self):
        self.screen.fill((255, 255, 255))
        for entity in self.entities:
            if entity.shape == 'circle':
                pixel_pos = [int(i * self.scale_factor) for i in entity.position]
                pygame.draw.circle(
                    self.screen,
                    pygame.Color(entity.color),
                    center=pixel_pos,
                    radius=int(entity.size * self.scale_factor)
                )
            else:
                rect = pygame.Rect(
                    (entity.position[0] - entity.size[0] / 2) * self.scale_factor,
                    (entity.position[1] - entity.size[1] / 2) * self.scale_factor,
                    entity.size[0] * self.scale_factor,
                    entity.size[1] * self.scale_factor,
                )
                pygame.draw.rect(
                    self.screen,
                    pygame.Color(entity.color),
                    rect=rect
                )
        return

    def reset(self):
        self._entities = []
        obs = self.get_observation("array")
        return obs, None

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    @property
    def scale_factor(self):
        return self._scale_factor

    @property
    def entities(self):
        return self._entities

    @property
    def engine(self):
        return self._engine

    @engine.setter
    def engine(self, value):
        self._engine = value

    def add_entity(self, entity):
        self._entities.append(entity)
        if entity.collision:
            self._engine.add_entity(entity)

    def remove_entity(self, entity_id):

        self._entities = [entity for entity in self._entities if entity.id != entity_id]

        if self._entities[entity_id].collision:
            self._engine.remove_entity(entity_id)

    def get_entities_by_type(self, entity_type):
        """Get a list of entities of a specified type."""
        return [entity for entity in self._entities if entity.__class__.__name__ == entity_type]

    def get_entity_position(self, entity_id):
        """Get the position of the entity with the specified ID."""
        for entity in self._entities:
            if entity.id == entity_id:
                entity.position = self._engine.get_entity_state(entity_id)[0]
                return entity.position
        raise ValueError(f"No entity with ID {entity_id} found.")

    def get_entity_velocity(self, entity_id):
        """Get the velocity of the entity with the specified ID."""
        for entity in self._entities:
            if entity.id == entity_id:
                entity.velocity = self._engine.get_entity_state(entity_id)[1]
                return entity.velocity
        raise ValueError(f"No entity with ID {entity_id} found.")

    def set_entity_velocity(self, entity_id, velocity):
        """Set the velocity of the entity with the specified ID."""
        for entity in self._entities:
            if entity.id == entity_id:
                self._engine.control_velocity(entity_id, velocity, self._dt)
                return
        raise ValueError(f"No entity with ID {entity_id} found.")

    def get_entity_by_id(self, entity_id):
        for entity in self._entities:
            if entity.id == entity_id:
                return entity
        raise ValueError(f"No entity with ID {entity_id} found.")

    def connect_to(self, entity1_id, entity2_id):
        entity1 = self.get_entity_by_id(entity1_id)
        entity2 = self.get_entity_by_id(entity2_id)
        distance = np.linalg.norm(entity1.position - entity2.position)
        if distance > 1.1 * (entity1.size + entity2.size):
            return False
        self._engine.add_joint(entity1_id, entity2_id, entity1.size + entity2.size)
        return True

    def disconnect_entities(self, entity1_id, entity2_id):
        try:
            entity1 = self.get_entity_by_id(entity1_id)
            entity2 = self.get_entity_by_id(entity2_id)
        except ValueError:
            return False
        self._engine.remove_joint(entity1_id, entity2_id)
        return True

    @staticmethod
    def sample_points(center, num_points, min_distance, shape='circle', size=None, max_attempts_per_point=10):
        def distance(p1, p2):
            return np.sqrt(np.sum((p1 - p2) ** 2, axis=1))

        points = []
        center = np.array(center)
        attempts = 0

        if shape == 'circle':
            if size is None or len(size) != 1:
                raise ValueError("For circle, size should be a list or tuple with one element: [radius].")
            radius = size[0] - min_distance
        elif shape == 'rectangle':
            if size is None or len(size) != 2:
                raise ValueError("For rectangle, size should be a list or tuple with two elements: [width, height].")
            width, height = size
            width -= 2 * min_distance
            height -= 2 * min_distance
        else:
            raise ValueError(f"Unsupported shape: {shape}")

        while len(points) < num_points and attempts < num_points * max_attempts_per_point:
            if shape == 'circle':
                # Generate random points within the bounding square and then filter to within the circle
                random_points = np.random.uniform(-radius, radius, size=(num_points, 2)) + center
                valid_mask = np.linalg.norm(random_points - center, axis=1) <= radius
                random_points = random_points[valid_mask]
            elif shape == 'rectangle':
                # Generate random points within the bounding rectangle
                random_points = np.random.uniform(-0.5, 0.5, size=(num_points, 2)) * np.array([width, height]) + center
            else:
                raise ValueError(f"Unsupported shape: {shape}")

            for point in random_points:
                if len(points) == 0 or np.all(distance(np.array(points), point) >= min_distance):
                    points.append(point)
                    if len(points) >= num_points:
                        break

            attempts += 1

        if len(points) < num_points:
            raise Exception(f"Warning: Could only place {len(points)} points out of {num_points} requested.")

        return np.array(points)


if __name__ == '__main__':
    env = GymnasiumEnvironmentBase("../../../config/env_config.json")

    # from modules.deployment.entity import Robot
    # import matplotlib.pyplot as plt
    # from scipy.signal import find_peaks
    #
    #
    # def calculate_performance_metrics(time_data, velocity_data, desired_velocity_x):
    #     # 上升时间
    #     try:
    #         rise_time_index = np.where(velocity_data >= 0.9 * desired_velocity_x)[0][0]
    #
    #         rise_time = time_data[rise_time_index]
    #     except:
    #         rise_time = 0
    #     # 峰值时间和最大超调量
    #     peaks, _ = find_peaks(velocity_data)
    #     peak_time = time_data[peaks[0]] if len(peaks) > 0 else None
    #     max_overshoot = ((velocity_data[peaks[0]] - desired_velocity_x) / desired_velocity_x) * 100 if len(
    #         peaks) > 0 else 0
    #
    #     # 稳态误差
    #     steady_state_value = velocity_data[-1]
    #     steady_state_error = desired_velocity_x - steady_state_value
    #
    #     return rise_time, peak_time, max_overshoot, steady_state_error
    #
    #
    # def generate_signals(signal_type, amplitude, frequency, duration, time_step):
    #     t = np.arange(0, duration, time_step)
    #     if signal_type == 'sine':
    #         signal = amplitude * np.sin(2 * np.pi * frequency * t)
    #     elif signal_type == 'square':
    #         signal = amplitude * np.sign(np.sin(2 * np.pi * frequency * t))
    #     else:
    #         raise ValueError(f"Unsupported signal type: {signal_type}")
    #     return t, signal
    #
    #
    # env = EnvironmentBase(1000, 1000, engine_type='Box2DEngine')
    # env.add_entity(Robot(0, np.array([500, 500]), 10.0))
    #
    # # 信号参数
    # amplitude = 50
    # frequency = 10
    # signal_type = 'square'  # 可以改为 'square' 来测试方波信号
    # time_step = 1 / 100  # 时间步长
    # total_time = 1  # 总模拟时间
    #
    # time_data, signal_data = generate_signals(signal_type, amplitude, frequency, total_time, time_step)
    # velocity_data = []
    # # env.set_entity_velocity(0, amplitude)
    #
    # for current_time, desired_velocity_x in zip(time_data, signal_data):
    #     env.update(time_step)
    #     desired_velocity = np.array([desired_velocity_x, 0.0])
    #     env.set_entity_velocity(0, desired_velocity)
    #
    #     # 获取并打印当前状态
    #     velocity = env.get_entity_velocity(0)
    #     velocity_data.append(velocity[0])
    #     print(f"Time: {current_time:.2f}, Velocity: {velocity}")
    #
    # # 计算动态性能指标
    # velocity_data = np.array(velocity_data)
    # rise_time, peak_time, max_overshoot, steady_state_error = calculate_performance_metrics(time_data, velocity_data,
    #                                                                                         amplitude)
    #
    # # 打印性能指标
    # print(f"Rise Time: {rise_time:.2f} s")
    # if peak_time:
    #     print(f"Peak Time: {peak_time:.2f} s")
    #     print(f"Maximum Overshoot: {max_overshoot:.2f} %")
    # print(f"Steady-State Error: {steady_state_error:.2f} m/s")
    #
    # # 绘图
    # plt.figure(figsize=(10, 6))
    # plt.plot(time_data, velocity_data, label='Velocity (x direction)')
    # plt.plot(time_data, signal_data, '--', label='Desired Signal')
    # plt.axvline(x=rise_time, color='g', linestyle='--', label='Rise Time')
    # if peak_time:
    #     plt.axvline(x=peak_time, color='b', linestyle='--', label='Peak Time')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Velocity (m/s)')
    # plt.title(f'Velocity over Time ({signal_type.capitalize()} Signal)')
    # plt.legend()
    # plt.grid(True)
    # plt.show()

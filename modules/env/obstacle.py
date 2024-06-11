import json

import geometry_msgs
import gym
import numpy as np
from typing import Union, List

import rospy
from code_llm.msg import Observations, ObjInfo
from geometry_msgs.msg import Twist, Vector3, Point
import pygame
from typing import Union, List

from gym import spaces
from std_srvs.srv import SetBool, SetBoolResponse


class Entity:
    # TODO: 改成gym的标准接口
    # TODO: 使用向量来大规模计算
    def __init__(self,
                 entity_id: int,
                 initial_position: Union[List[float], tuple, np.ndarray],
                 size: Union[List[float], tuple, np.ndarray, float],
                 color: str,
                 collision: bool = False,
                 moveable: bool = False):
        """
        Initialize a new entity with the given parameters.
        Connectors means the entity is connected to another entity.

        :param entity_id: Unique identifier for the entity.
        :param initial_position: Initial position of the entity.
        :param size: Size of the entity.
        :param color: Color of the entity.
        :param collision: Indicates if the entity has collision enabled.
        :param moveable: Indicates if the entity is moveable.
        """
        self.__id = entity_id
        self.__position = np.array(initial_position, dtype=float)
        self.__history = [self.__position.copy()]
        self.__size = size
        self.__mass = 1.0
        self.__connector: List[Entity] = []
        self.__color: str = color
        self.__alpha: float = 0.7
        self.__collision: bool = collision
        self.__moveable: bool = moveable
        self.__velocity = np.array([0.0, 0.0], dtype=float)
        self.__max_speed = 2.0
        self.__shape = 'circle' if isinstance(size, float) else 'rectangle'

    @property
    def position(self) -> np.ndarray:
        """Get the current position of the entity."""
        return self.__position

    @position.setter
    def position(self, value: Union[List[float], tuple, np.ndarray]):
        """Set a new position for the entity."""
        if self.__moveable:
            self.__position = np.array(value, dtype=float)
            self.__history.append(self.__position.copy())
        else:
            raise ValueError("Entity is not moveable.")

    @property
    def history(self):
        """Get the historical positions of the entity."""
        return np.array(self.__history)

    @property
    def size(self) -> Union[List[float], tuple, np.ndarray, float]:
        """Get the size of the entity."""
        return self.__size

    @property
    def id(self) -> int:
        """Get the unique identifier of the entity."""
        return self.__id

    @property
    def color(self) -> str:
        """Get the color of the entity."""
        return self.__color

    @property
    def shape(self) -> str:
        """Get the shape of the entity."""
        return self.__shape

    @color.setter
    def color(self, value: str):
        """Set a new color for the entity."""
        if isinstance(value, str):
            self.__color = value
        else:
            raise ValueError("Color must be a string.")

    @property
    def alpha(self) -> float:
        """Get the transparency level of the entity."""
        return self.__alpha

    @alpha.setter
    def alpha(self, value: float):
        """Set a new transparency level for the entity."""
        if 0.0 <= value <= 1.0:
            self.__alpha = value
        else:
            raise ValueError("Alpha must be between 0.0 and 1.0.")

    @property
    def collision(self) -> bool:
        """Get the collision status of the entity."""
        return self.__collision

    @collision.setter
    def collision(self, value: bool):
        """Set the collision status for the entity."""
        if isinstance(value, bool):
            self.__collision = value
        else:
            raise ValueError("Collision must be a boolean.")

    @property
    def moveable(self) -> bool:
        """Get the moveable status of the entity."""
        return self.__moveable

    @moveable.setter
    def moveable(self, value: bool):
        """Set the moveable status for the entity."""
        if isinstance(value, bool):
            self.__moveable = value
        else:
            raise ValueError("Moveable must be a boolean.")

    def add_connector(self, connector):
        """
        Add a connector to the entity.

        :param connector: The connector to add.
        """
        self.__connector.append(connector)

    def remove_connector(self, connector):
        """
        Remove a connector from the entity.

        :param connector: The connector to remove.
        """
        if connector in self.__connector:
            self.__connector.remove(connector)
        else:
            raise ValueError("Connector not found.")

    def get_connectors(self) -> List:
        """
        Get the list of connectors.

        :return: List of connectors.
        """
        return self.__connector

    @property
    def velocity(self) -> np.ndarray:
        """Get the current velocity of the robot."""
        return self.__velocity

    @velocity.setter
    def velocity(self, new_velocity: Union[List[float], tuple, np.ndarray]):
        """Set a new velocity for the robot."""
        self.__velocity = np.array(new_velocity, dtype=float)

    @property
    def max_speed(self) -> float:
        """Get the maximum speed of the leader."""
        return self.__max_speed

    @max_speed.setter
    def max_speed(self, value: float):
        """Set a new maximum speed for the leader."""
        self.__max_speed = value

    @property
    def mass(self) -> float:
        """Get the mass of the entity."""
        return self.__mass

    @mass.setter
    def mass(self, value: float):
        """Set the mass of the entity."""
        if value > 0:
            self.__mass = value
        else:
            raise ValueError("Mass must be a positive value.")

    def move(self, dt):
        """Move the entity based on its velocity, ensuring no collisions."""
        if not self.moveable:
            return

        if self.__connector:
            total_mass = self.mass + sum(connector.mass for connector in self.__connector)
            weighted_velocity = (self.velocity * self.mass + sum(
                connector.velocity * connector.mass for connector in self.__connector)) / total_mass
            self.__velocity = weighted_velocity
        new_position = self.position + self.velocity * dt
        self.position = new_position

    def check_circle_rectangle_collision(self, rect, circle_position):
        """Check if a circle and rectangle are colliding."""
        circle_center = circle_position
        rect_center = rect.position
        rect_half_size = np.array(rect.size) / 2

        # Find the closest point to the circle within the rectangle
        closest_point = np.clip(circle_center, rect_center - rect_half_size, rect_center + rect_half_size)

        # Calculate the distance between the circle's center and this closest point
        distance = np.linalg.norm(circle_center - closest_point)

        return distance < self.size


class Obstacle(Entity):
    def __init__(self, obstacle_id, initial_position, size):
        super().__init__(obstacle_id,
                         initial_position,
                         size,
                         color="gray",
                         collision=True,
                         moveable=False)


class Landmark(Entity):
    def __init__(self, landmark_id, initial_position, size, color):
        super().__init__(landmark_id,
                         initial_position,
                         size,
                         color=color,
                         collision=False,
                         moveable=False)


class Robot(Entity):
    def __init__(self, robot_id, initial_position, size, color='green'):
        super().__init__(robot_id,
                         initial_position,
                         size,
                         color=color,
                         collision=True,
                         moveable=True)

    def connected_to(self, entity):
        """
        Connect the robot to another entity.
        :param entity: The entity to connect to.
        """
        self.add_connector(entity)
        entity.add_connector(self)

    def disconnect_from(self, entity):
        """
        Disconnect the robot from another entity.
        :param entity: The entity to disconnect from.
        """
        self.remove_connector(entity)
        entity.remove_connector(self)


class PushableObject(Entity):
    def __init__(self, object_id, initial_position, size):
        super().__init__(object_id,
                         initial_position,
                         size,
                         color='yellow',
                         collision=True,
                         moveable=True)


class Leader(Entity):
    def __init__(self, leader_id, initial_position, size):
        super().__init__(leader_id, initial_position, size, color='red')


class EnvironmentBase:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.entities = []

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
        for entity in self.entities:
            entity.move(dt)

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


class SimulationEnvironment(EnvironmentBase):
    def __init__(self, width: int, height: int, data_file: str = None, output_file: str = "output.json"):
        super().__init__(width, height)
        self.data_file = data_file
        self.output_file = output_file
        self.generated_entities = []
        if self.data_file:
            self.add_entities_from_config()

    def add_entities_from_config(self):
        def generate_random_position(entity_size, entity_shape, max_attempts=1000):
            attempts = 0
            while attempts < max_attempts:
                position = np.random.uniform([entity_size, entity_size],
                                             [self.width - entity_size, self.height - entity_size])
                if not any(self.check_collision(position, entity_size, entity_shape, other) for other in self.entities):
                    return position
                attempts += 1
            raise ValueError(f"Failed to generate non-colliding position after {max_attempts} attempts.")

        def add_specified_entities(entity_type, entity_class, color=None):
            nonlocal entity_id
            for entity_data in data["entities"][entity_type]["specified"]:
                entity = entity_class(entity_id, entity_data["position"], entity_data["size"])
                if color:
                    entity.color = entity_data.get("color", color)
                self.add_entity(entity)
                self.generated_entities.append(entity)
                entity_id += 1

        with open(self.data_file, 'r') as file:
            data = json.load(file)

        entity_id = 0

        add_specified_entities("leader", Leader, "red")
        add_specified_entities("obstacle", Obstacle)
        add_specified_entities("landmark", Landmark)
        add_specified_entities("pushable_object", PushableObject)
        add_specified_entities("robot", Robot, "green")

        # Add remaining robots
        for _ in range(data["entities"]["robot"]["count"] - len(data["entities"]["robot"]["specified"])):
            size = 5.0
            shape = 'circle'
            position = generate_random_position(size, shape)
            robot = Robot(entity_id, position, size, "green")
            self.add_entity(robot)
            self.generated_entities.append(robot)
            entity_id += 1

        self.add_walls()

    def check_collision(self, position, size, shape, other):
        if shape == 'circle':
            if other.shape == 'circle':
                return np.linalg.norm(position - other.position) < (size + other.size)
            else:
                return self.check_circle_rectangle_collision(other, position, size)
        else:
            if other.shape == 'circle':
                return self.check_circle_rectangle_collision(other, position, size)
            else:
                rect1 = pygame.Rect(position[0] - size[0] / 2, position[1] - size[1] / 2, size[0], size[1])
                rect2 = pygame.Rect(other.position[0] - other.size[0] / 2, other.position[1] - other.size[1] / 2,
                                    other.size[0], other.size[1])
                return rect1.colliderect(rect2)

    def check_circle_rectangle_collision(self, rect, circle_position, circle_radius):
        circle_center = circle_position
        rect_center = rect.position
        rect_half_size = np.array(rect.size) / 2

        # Find the closest point to the circle within the rectangle
        closest_point = np.clip(circle_center, rect_center - rect_half_size, rect_center + rect_half_size)

        # Calculate the distance between the circle's center and this closest point
        distance = np.linalg.norm(circle_center - closest_point)

        return distance < circle_radius

    def save_entities_to_file(self):
        entities_data = []
        for entity in self.generated_entities:
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


class Manager:
    def __init__(self, env: EnvironmentBase):
        self.env = env

        rospy.init_node('simulation_manager', anonymous=True)
        rospy.Subscriber("/leader/velocity", Twist, self.leader_velocity_callback)
        # self._reset_service = rospy.Service(
        #     "/reset_environment", SetBool, self.reset_environment_callback
        # )
        self.observation_publisher = rospy.Publisher(f"observation", Observations, queue_size=1)

        self._pub_list = []
        self._robots = env.get_entities_by_type(Robot)
        robot_start_index = self._robots[0].id
        robot_end_index = self._robots[-1].id
        rospy.set_param("robot_start_index", robot_start_index)
        rospy.set_param("robot_end_index", robot_end_index)
        for i in range(robot_start_index, robot_end_index + 1):
            rospy.Subscriber(
                f"/robot_{i}/velocity", Twist, self.velocity_callback, callback_args=i
            )
        # self._timer = rospy.Timer(rospy.Duration(0.01), self.publish_observations)
        self.received_velocity = {robot.id: False for robot in self._robots}  # 初始化接收状态字典

    def velocity_callback(self, data: geometry_msgs.msg.Twist, i):
        """
        velocity_callback is a callback function for the velocity topic.
        """
        self.env.set_entity_velocity(entity_id=i, new_velocity=[data.linear.x * 100, data.linear.y * 100])
        # print(f"Robot {i} velocity: {data.linear.x}, {data.linear.y}")
        self.received_velocity[i] = True  # 更新机器人接收状态

        # 检查所有机器人是否都已接收到速度信息
        if all(self.received_velocity.values()):
            print("All robots have received initial velocity. Initialization successful.")

    def leader_velocity_callback(self, data: Twist):
        leader = self.env.get_entities_by_type(Leader)[0]
        leader.speed = np.array([data.linear.x, data.linear.y])

    def connect_entities(self, entity1_id, entity2_id):
        self.env.connect_entities(entity1_id, entity2_id)

    def disconnect_entities(self, entity1_id, entity2_id):
        self.env.disconnect_entities(entity1_id, entity2_id)

    def publish_observations(self):
        observation = self.env.get_observation()
        observations_msg = Observations()
        observations_msg.observations = []
        for entity_id, entity in observation.items():
            obj_info = ObjInfo()
            obj_info.id = entity_id
            obj_info.position = Point(x=entity["position"][0], y=entity["position"][1], z=0)
            obj_info.velocity = Twist(linear=Vector3(x=entity["velocity"][0], y=entity["velocity"][1], z=0),
                                      angular=Vector3(x=0, y=0, z=0))
            obj_info.radius = entity["size"] if isinstance(entity["size"], float) else 0.0
            obj_info.type = entity["type"]
            observations_msg.observations.append(obj_info)
        self.observation_publisher.publish(observations_msg)


# def reset_environment_callback(self):
#     pass


# class GymEnvironment():
#     def __init__(self, config_file=None, width=1000, height=1000):
#         super(GymEnvironment, self).__init__()
#         self.env = SimulationEnvironment(width, height, data_file=config_file)
#         self.current_step = 0
#
#     def reset(self):
#         self.env = SimulationEnvironment(self.env.width, self.env.height, data_file=self.env.data_file)
#         self.current_step = 0
#         return self._get_observation()
#
#     def step(self, action, dt):
#         self.env.set_entity_velocity(entity_id=0, new_velocity=action)
#         self.env.update(dt=dt)
#         self.current_step += 1
#         observation = self._get_observation()
#         reward = self._get_reward()
#         done = self._is_done()
#         return observation, reward, done, {}
#
#     def _get_observation(self):
#         obs = {}
#         for entity in self.env.entities:
#             obs[entity.id] = {
#                 "position": entity.position,
#                 "velocity": entity.velocity,
#                 "type": entity.__class__.__name__
#             }
#         return obs
#
#     def _get_reward(self):
#         # Define your own reward function here
#         return 0.0
#
#     def _is_done(self):
#         # Define your own done condition here
#         return False
#
#     def render(self, mode='human', screen=None):
#         if mode == 'human':
#             self.env.draw(screen)


def main(config_file=None):
    pygame.init()

    env = SimulationEnvironment(1000, 1000, data_file=config_file)
    screen = pygame.display.set_mode((env.width, env.height))
    clock = pygame.time.Clock()

    manager = Manager(env)
    manager.publish_observations()

    running = True
    rate = rospy.Rate(10)
    env.save_entities_to_file()
    draw_counter = 0
    draw_frequency = 1  # 每5帧绘图一次
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        dt = clock.tick(50) / 1000
        env.update(dt)
        if draw_counter % draw_frequency == 0:
            env.draw(screen)
        manager.publish_observations()
        # rospy.spin()
        draw_counter += 1

        rate.sleep()

    pygame.quit()
    env.save_entities_to_file()


if __name__ == "__main__":
    file = 'env_config/base.json'
    main(config_file=file)

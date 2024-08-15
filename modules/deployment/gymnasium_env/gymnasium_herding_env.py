from typing import Optional
from typing import TYPE_CHECKING, Any, Generic, SupportsFloat, TypeVar
from modules.deployment.entity import Robot, Sheep
from modules.deployment.gymnasium_env.gymnasium_base_env import GymnasiumEnvironmentBase
from modules.deployment.gymnasium_env.utils import *

ObsType = TypeVar("ObsType")
ActType = TypeVar("ActType")
RenderFrame = TypeVar("RenderFrame")


class GymnasiumHerdingEnvironment(GymnasiumEnvironmentBase):

    def __init__(self, data_file: str):
        super().__init__(data_file)

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed, options=options)
        self.entities = []
        self.init_entities()
        obs = self.get_observation("array")
        infos = self.get_observation("dict")
        return obs, infos

    def init_entities(self):
        # wall_width = 10
        # wall = Wall(wall_id=0,
        #             initial_position=(0.5 * self.width, 0.5 * wall_width),
        #             size=(self.width, wall_width))
        # self.add_entity(wall)
        # wall = Wall(wall_id=1,
        #             initial_position=(0.5 * self.width, self.height - 0.5 * wall_width),
        #             size=(self.width, wall_width))
        # self.add_entity(wall)
        # wall = Wall(wall_id=2,
        #             initial_position=(0.5 * wall_width, 0.5 * self.height),
        #             size=(wall_width, self.height))
        # self.add_entity(wall)
        # wall = Wall(wall_id=3,
        #             initial_position=(self.width - 0.5 * wall_width, 0.5 * self.height),
        #             size=(wall_width, self.height))
        # self.add_entity(wall)
        # left_wall = Wall(wall_id=4,
        #                  initial_position=(200, 400),
        #                  size=(0.45 * self.width, wall_width))
        # self.add_entity(left_wall)
        # right_wall = Wall(wall_id=5,
        #                   initial_position=(800, 400),
        #                   size=(0.45 * self.width, wall_width))
        # self.add_entity(right_wall)

        entity_id = 0

        robot_size = self.data["entities"]["robot"]["size"]
        shape = self.data["entities"]["robot"]["shape"]
        color = self.data["entities"]["robot"]["color"]

        for i in range(self.num_robots):
            # Calculate the angle for even distribution along the boundary
            angle = (entity_id / self.num_robots) * 2 * np.pi

            # Determine x and y coordinates based on angle
            if 0 <= angle < np.pi / 2:  # Top side
                x = 1 * angle / (np.pi / 2)
                y = 0
            elif np.pi / 2 <= angle < np.pi:  # Right side
                x = 1
                y = 1 * (angle - np.pi / 2) / (np.pi / 2)
            elif np.pi <= angle < 3 * np.pi / 2:  # Bottom side
                x = 1 * (1 - (angle - np.pi) / (np.pi / 2))
                y = 1
            else:  # Left side
                x = 0
                y = 1 * (1 - (angle - 3 * np.pi / 2) / (np.pi / 2))

            dog = Robot(robot_id=entity_id,
                        initial_position=np.array([x, y]),
                        size=robot_size,
                        color=color)
            self.add_entity(dog)
            entity_id += 1

        sheep_size = self.data.get("entities").get("sheep",{}).get("size", 0.15)
        shape = self.data.get("entities").get("sheep",{}).get("shape", "circle")
        color = self.data.get("entities").get("sheep",{}).get("color", "blue")

        for i in range(self.num_robots):
            position = sample_point(zone_center=[0, 0], zone_shape='rectangle', zone_size=[self.width, self.height],
                                    robot_size=sheep_size, robot_shape=shape, min_distance=sheep_size,
                                    entities=self.entities)
            sheep = Sheep(prey_id=entity_id,
                          initial_position=position,
                          size=sheep_size)
            self.add_entity(sheep)
            entity_id += 1

    def update(self, action=ActType):
        super().step(action)

        for entity in self.entities:
            if isinstance(entity, Sheep):
                # 获取邻居羊群和机器人列表
                flock = [e for e in self.entities if isinstance(e, Sheep) and e != entity]
                robots = [e for e in self.entities if isinstance(e, Robot)]
                speed = entity.calculate_velocity(flock, robots)
                self.set_entity_velocity(entity.id, speed)


if __name__ == "__main__":

    import time
    import rospy

    from modules.deployment.utils.manager import Manager

    env = GymnasiumHerdingEnvironment("../../../config/env_config.json")

    obs, infos = env.reset()
    manager = Manager(env)
    manager.publish_observations(infos)
    rate = rospy.Rate(env.FPS)

    start_time = time.time()  # 记录起始时间
    frame_count = 0  # 初始化帧数计数器

    while True:
        # action = manager.robotID_velocity
        action = {}
        # manager.clear_velocity()
        obs, reward, termination, truncation, infos = env.step(action=action)
        env.render()
        manager.publish_observations(infos)
        rate.sleep()

        frame_count += 1  # 增加帧数计数器
        current_time = time.time()  # 获取当前时间
        elapsed_time = current_time - start_time  # 计算已过去的时间

        # 当达到1秒时，计算并打印FPS，然后重置计数器和时间
        if elapsed_time >= 1.0:
            fps = frame_count / elapsed_time
            print(f"FPS: {fps:.2f}")  # 打印FPS，保留两位小数
            frame_count = 0  # 重置帧数计数器
            start_time = current_time  # 重置起始时间戳
    print("Simulation completed successfully.")

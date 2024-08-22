from typing import Any, SupportsFloat, TypeVar, Optional

from modules.deployment.entity import Landmark, Leader, Obstacle, PushableObject, Robot
from modules.deployment.utils.sample_point import *
from modules.deployment.gymnasium_env.gymnasium_base_env import GymnasiumEnvironmentBase


ObsType = TypeVar("ObsType")
ActType = TypeVar("ActType")


class GymnasiumFlockingEnvironment(GymnasiumEnvironmentBase):
    def __init__(self, data_file: str = None):
        super().__init__(data_file)

    def add_entities_from_config(self):

        def add_specified_entities(entity_type, entity_class, color=None):
            nonlocal entity_id
            for entity_data in self.data["entities"][entity_type]["specified"]:
                entity = entity_class(entity_id, entity_data["position"], entity_data["size"])
                if color:
                    entity.color = entity_data.get("color", color)
                self.add_entity(entity)
                entity_id += 1

        entity_id = 0

        add_specified_entities("leader", Leader, "red")
        add_specified_entities("obstacle", Obstacle)
        add_specified_entities("landmark", Landmark)
        add_specified_entities("pushable_object", PushableObject)
        add_specified_entities("robot", Robot, "green")

        # Add remaining robots
        if "count" in self.data["entities"]["robot"]:
            robot_size = self.data["entities"]["robot"]["size"]
            robot_num = self.data["entities"]["robot"]["count"] - len(self.data["entities"]["robot"]["specified"])
            shape = self.data["entities"]["robot"]["shape"]
            color = self.data["entities"]["robot"]["color"]

            for i in range(robot_num):
                position = sample_point(zone_center=[0, 0], zone_shape='rectangle', zone_size=[self.width, self.height],
                                        robot_size=robot_size, robot_shape=shape, min_distance=robot_size,
                                        entities=self.entities)
                robot = Robot(entity_id, position, robot_size, color=color)
                self.add_entity(robot)
                entity_id += 1

    def step(
            self, action: ActType
    ) -> tuple[ObsType, SupportsFloat, bool, bool, dict[str, Any]]:

        return super().step(action)

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed, options=options)
        self.entities = []
        self.moveable_agents = {}
        self.add_entities_from_config()
        obs = self.get_observation("array")
        infos = self.get_observation("dict")
        return obs, infos


if __name__ == '__main__':

    import time
    import rospy

    from modules.deployment.utils.manager import Manager

    env = GymnasiumFlockingEnvironment("../../../config/env_config.json")
    obs, infos = env.reset()
    print(env.moveable_agents)
    manager = Manager(env)
    manager.publish_observations(infos)
    rate = rospy.Rate(env.FPS)

    start_time = time.time()  # 记录起始时间
    frame_count = 0  # 初始化帧数计数器
    current_time = time.time()
    while current_time - start_time < 120:
        action = manager.robotID_velocity
        manager.clear_velocity()
        # print(action)
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

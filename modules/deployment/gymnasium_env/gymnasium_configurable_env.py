from modules.deployment.entity import Landmark, Leader, Obstacle, PushableObject, Robot
from modules.deployment.gymnasium_env.base_env import GymnasiumEnvironmentBase
from modules.deployment.gymnasium_env.utils import *
from typing import Optional
from typing import TYPE_CHECKING, Any, Generic, SupportsFloat, TypeVar

ObsType = TypeVar("ObsType")
ActType = TypeVar("ActType")
RenderFrame = TypeVar("RenderFrame")


class GymnasiumConfigurableEnvironment(GymnasiumEnvironmentBase):
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
            size = self.data["entities"]["robot"]["size"]
            robot_num = self.data["entities"]["robot"]["count"] - len(self.data["entities"]["robot"]["specified"])
            shape = self.data["entities"]["robot"]["shape"]
            color = self.data["entities"]["robot"]["color"]
            position = sample_points(center=[0, 0], num_points=robot_num, min_distance=size * 2, shape='rectangle',
                                     size=[self.width, self.height])
            for i in range(robot_num):
                robot = Robot(entity_id, position[i], size, color=color)
                self.add_entity(robot)
                entity_id += 1

    def step(
            self, action: ActType
    ) -> tuple[ObsType, SupportsFloat, bool, bool, dict[str, Any]]:

        return super().step(action)

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed, options=options)
        self.entities = []
        self.add_entities_from_config()
        obs = self.get_observation("array")
        infos = self.get_observation("dict")
        return obs, infos
        # self.agents = [f"agent_{i}" for i in range(self.num_agent)]
        # self.agent_name_mapping = dict(zip(self.agents, list(range(self.num_agent))))
        # self._agent_selector = agent_selector(self.agents)
        # self._agent_selector.reinit(self.agents)
        # self.agent_selection = self._agent_selector.next()


if __name__ == '__main__':
    env = GymnasiumConfigurableEnvironment("../../../config/env_config.json")
    obs, infos = env.reset()

    from modules.deployment.utils.manager import Manager
    manager = Manager(env)
    manager.publish_observations(infos)
    import rospy

    rate = rospy.Rate(env.FPS)
    while True:
        action = manager.robotID_velocity
        manager.clear_velocity()
        obs, reward, termination, truncation, infos = env.step(action=action)
        env.render()
        manager.publish_observations(infos)
        rate.sleep()
    print("Simulation completed successfully.")
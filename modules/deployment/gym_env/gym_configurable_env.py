from modules.deployment.entity import Landmark, Leader, Obstacle, PushableObject, Robot
from modules.deployment.gym_env.base_env import GymnasiumEnvironmentBase
from modules.deployment.gym_env.utils import *
# from pettingzoo.utils import agent_selector, wrappers

class GymnasiumConfigurableEnvironment():
    def __init__(self, data_file: str = None):
        self.env = GymnasiumEnvironmentBase(data_file)
        self.generated_entities = []
        self.has_reset = False
        self.num_agent = 0

    def add_entities_from_config(self):
        def add_specified_entities(entity_type, entity_class, color=None):
            nonlocal entity_id
            for entity_data in self.env.data["entities"][entity_type]["specified"]:
                entity = entity_class(entity_id, entity_data["position"], entity_data["size"])
                if color:
                    entity.color = entity_data.get("color", color)
                self.env.add_entity(entity)
                self.generated_entities.append(entity)
                entity_id += 1
            if entity_type in ["robot", "leader"]:
                self.num_agent += 1
        entity_id = 0

        add_specified_entities("leader", Leader, "red")
        add_specified_entities("obstacle", Obstacle)
        add_specified_entities("landmark", Landmark)
        add_specified_entities("pushable_object", PushableObject)
        add_specified_entities("robot", Robot, "green")

        # Add remaining robots
        for _ in range(self.env.data["entities"]["robot"]["count"] - len(self.env.data["entities"]["robot"]["specified"])):
            size = self.env.data["entities"]["robot"]["size"]
            shape = self.env.data["entities"]["robot"]["shape"]
            color = self.env.data["entities"]["robot"]["color"]
            position = generate_random_position(size, shape, self.env.entities, self.env.width, self.env.height)
            robot = Robot(entity_id, position, size, color=color)
            self.env.add_entity(robot)
            self.generated_entities.append(robot)
            entity_id += 1
            self.num_agent += 1

    def reset(self):
        self.has_reset = True
        self.env.reset()
        self.generated_entities = []
        self.add_entities_from_config()
        # self.agents = [f"agent_{i}" for i in range(self.num_agent)]
        # self.agent_name_mapping = dict(zip(self.agents, list(range(self.num_agent))))
        # self._agent_selector = agent_selector(self.agents)
        # self._agent_selector.reinit(self.agents)
        # self.agent_selection = self._agent_selector.next()

    def step(self):
        obs, reward, termination, truncation, infos = self.env.step()
        return obs, reward, termination, truncation, infos

    def render(self):
        return self.env.render()

    def close(self):
        if self.has_reset:
            self.env.close()

if __name__ == '__main__':
    env = GymnasiumConfigurableEnvironment("../../../config/env_config.json")
    env.reset()
    from modules.deployment.utils.manager import Manager
    manager = Manager(env.env)
    manager.publish_observations()
    import rospy
    rate = rospy.Rate(100)

    import time
    start_time = time.time()
    while int(time.time() - start_time ) < 1000:
        env.step()
        manager.publish_observations()
        rate.sleep()
    env.close()

from modules.deployment.entity import Robot, Obstacle, Leader, Landmark, PushableObject
from modules.deployment.gymnasium_env.gymnasium_base_env import GymnasiumEnvironmentBase
from modules.deployment.utils.sample_point import *
from modules.deployment.engine import OmniEngine


class GymnasiumRealEnvironment(GymnasiumEnvironmentBase):
    def __init__(self, data_file: str = None):
        super().__init__(data_file=data_file)
        self.engine = OmniEngine()

    def init_entities(self):
        def add_specified_entities(entity_type, entity_class, color=None):
            for entity_data in self.data["entities"][entity_type]["specified"]:
                entity_position = np.array(entity_data["position"])
                entity = entity_class(entity_data['id'], entity_position, entity_data["size"])
                if color:
                    entity.color = entity_data.get("color", color)
                self.add_entity(entity)
                self.entities.append(entity)

        add_specified_entities("leader", Leader, "red")
        add_specified_entities("obstacle", Obstacle)
        add_specified_entities("landmark", Landmark)
        add_specified_entities("pushable_object", PushableObject)
        add_specified_entities("robot", Robot, "green")

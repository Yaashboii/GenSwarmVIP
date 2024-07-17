import json

import numpy as np
import pygame

from modules.deployment.entity.landmark import Landmark
from modules.deployment.entity.leader import Leader
from modules.deployment.entity.obstacle import Obstacle
from modules.deployment.entity.pushable_object import PushableObject
from modules.deployment.entity.robot import Robot
from modules.deployment.env.base_env import EnvironmentBase


class RealEnvironment(EnvironmentBase):
    def __init__(self, width: int, height: int, data_file: str = None, output_file: str = "output.json"):
        super().__init__(width, height, engine_type='Omni_Engine')
        self.data_file = data_file
        self.output_file = output_file
        self.generated_entities = []

        if self.data_file:
            with open(self.data_file, 'r') as file:
                self.data = json.load(file)
            self.add_entities_from_config()

    def add_entities_from_config(self):
        def add_specified_entities(entity_type, entity_class, color=None):
            for entity_data in data["entities"][entity_type]["specified"]:
                entity_position = np.array(entity_data["position"]) * 100 + np.array([500, 500])
                entity = entity_class(entity_data['id'], entity_position, entity_data["size"])
                if color:
                    entity.color = entity_data.get("color", color)
                self.add_entity(entity)
                self.generated_entities.append(entity)

        with open(self.data_file, 'r') as file:
            data = json.load(file)

        add_specified_entities("leader", Leader, "red")
        add_specified_entities("obstacle", Obstacle)
        add_specified_entities("landmark", Landmark)
        add_specified_entities("pushable_object", PushableObject)
        add_specified_entities("robot", Robot, "green")

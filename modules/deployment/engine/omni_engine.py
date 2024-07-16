from modules.deployment.engine.engine import Engine
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np


class OmniEngine(Engine):
    def __init__(self):
        super().__init__()
        self.type_mapping = {'robot': 'VSWARM', 'obstacle': 'OBSTACLE'}
        self.subscribers = []

    def pose_callback(self, msg, args):
        entity_id, entity_type = args
        position = np.array([msg.pose.position.x, msg.pose.position.y]) * 1000 + np.array([500, 500])
        self.set_position(entity_id, position)
        print(f"Setting position of entity {entity_id} to {position}")

    def twist_callback(self, msg, args):
        entity_id, entity_type = args
        velocity = np.array([msg.twist.linear.x, msg.twist.linear.y])
        entity_id = int(entity_id)
        self.set_velocity(entity_id, velocity)


    def generate_subscribers(self, entity_id, entity_type):
        pose_topic = f"/vrpn_client_node/{entity_type.upper()}{entity_id}/pose"
        twist_topic = f"/vrpn_client_node/{entity_type.upper()}{entity_id}/twist"

        self.subscribers.append(
            rospy.Subscriber(pose_topic, PoseStamped, self.pose_callback, callback_args=(entity_id, entity_type)))
        self.subscribers.append(
            rospy.Subscriber(twist_topic, TwistStamped, self.twist_callback, callback_args=(entity_id, entity_type)))

    def generate_all_subscribers(self):
        for entity_id, entity in self._entities.items():
            a = entity.__class__.__name__.lower()
            self.generate_subscribers(entity_id=entity_id,
                                      entity_type=self.type_mapping[a])

    def step(self, delta_time: float):
        if len(self.subscribers) == 0:
            self.generate_all_subscribers()
        rospy.sleep(delta_time)

    def apply_force(self, entity_id: int, force: np.ndarray):
        print(f"Applying force {force} to entity {entity_id}")

    def control_velocity(self, entity_id, desired_velocity, dt=None):
        print(f"Controlling velocity of entity {entity_id} to {desired_velocity}")

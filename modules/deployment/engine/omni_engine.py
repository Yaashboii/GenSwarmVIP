import json

from .base_engine import Engine
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import socket
import os
import time
import threading

from scipy.spatial.transform import Rotation as R

from modules.deployment.utils.mqtt_pub import MqttClientThread


class OmniEngine(Engine):
    def __init__(self):
        super().__init__()
        self.type_mapping = {'robot': 'VSWARM', 'obstacle': 'OBSTACLE', 'prey': 'PREY'}
        self.subscribers = []
        self.mqtt_client = self.start_up_mqtt_thread()

    def start_up_mqtt_thread(self):
        broker_ip = "10.0.2.66"
        port = 1883
        keepalive = 60  # 与代理通信之间允许的最长时间段（以秒为单位）
        client_id = f'{self.__class__.__name__}'  # 客户端id不能重复

        try:
            broker = os.environ['REMOTE_SERVER']
        except KeyError:
            broker = broker_ip

        net_status = -1
        while net_status != 0:
            net_status = os.system(f"ping -c 4 {broker}")
            time.sleep(2)

        # 启动MQTT客户端线程
        mqtt_client_instance = MqttClientThread(broker=broker, port=port, keepalive=keepalive, client_id=client_id)
        mqtt_thread = threading.Thread(target=mqtt_client_instance.run)
        mqtt_thread.start()
        return mqtt_client_instance

    def pose_callback(self, msg, args):
        entity_id, entity_type = args
        position = np.array([msg.pose.position.x, msg.pose.position.y])
        self.set_position(entity_id, position)
        print(f"update position of omni bot {entity_id} to {position}")
        quaternion = np.array(
            [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
             msg.pose.orientation.w])

        rot_mat = R.from_quat(quaternion).as_matrix()
        # 解出欧拉角（RPY顺序）
        euler = np.array(R.from_matrix(rot_mat).as_euler('xyz', degrees=False))
        self.set_yaw(entity_id, euler[2])

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
        # for entity in self._entities:
        #     self.control_yaw(entity, desired_yaw=0)
        rospy.sleep(delta_time)

    def apply_force(self, entity_id: int, force: np.ndarray):
        print(f"Failed Applying force {force} to entity {entity_id} at omni bot")

    def control_velocity(self, entity_id, desired_velocity, dt=None):
        print(f"Failed Controlling velocity of entity {entity_id} to {desired_velocity} at omni bot")

    # def set_velocity(self, entity_id, velocity):
    #     json_msg = {
    #         "x": velocity[0],
    #         "y": velocity[1],
    #         "theta": velocity[2]
    #     }
    #
    #     json_str = json.dumps(json_msg)
    #     self.mqtt_client.publish(f'/VSWARM{entity_id}_robot/motion', json_str.encode('utf-8'))

    def control_yaw(self, entity_id, desired_yaw, dt=None):
        yaw_error = desired_yaw - self._entities[entity_id].yaw
        if yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        if yaw_error < -np.pi:
            yaw_error += 2 * np.pi
        kp = 0.8
        json_msg = {
            "x": 0,
            "y": 0,
            "theta": yaw_error * kp
        }
        print(f"yaw error is {yaw_error}")
        json_str = json.dumps(json_msg)
        self.mqtt_client.publish(f'/VSWARM{entity_id}_robot/motion', json_str.encode('utf-8'))

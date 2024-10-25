import unittest
from unittest.mock import MagicMock, patch
import json
import numpy as np
import rospy
from sensor_msgs.msg import Joy  # 新增导入 Joy 消息类型
from modules.deployment.engine.omni_engine import OmniEngine


class TestOmniEngine(unittest.TestCase):
    @patch(
        "modules.deployment.engine.omni_engine.MqttClientThread"
    )  # Mock MQTT client thread
    def setUp(self, MockMqttClientThread):
        self.mock_mqtt_client = MockMqttClientThread.return_value
        self.omni_engine = OmniEngine()
        self.omni_engine.mqtt_client = self.mock_mqtt_client  # Set mock MQTT client
        self.mock_mqtt_client.publish = MagicMock()

    # @patch('rospy.Subscriber')
    # def test_generate_subscribers(self, MockSubscriber):
    #     # Generate subscribers for an entity and verify
    #     entity_id = 1
    #     entity_type = 'robot'
    #     self.omni_engine.generate_subscribers(entity_id, entity_type)
    #     self.assertEqual(len(self.omni_engine.subscribers), 2)
    #     MockSubscriber.assert_called()  # Ensure subscribers are created

    # def test_joy_callback_updates_input(self):
    #     # Test joystick callback updates the joy input correctly
    #     joy_msg = Joy()
    #     joy_msg.axes = [0.5, -0.5, 0.0, 0.3]  # Example joystick input
    #     self.omni_engine.joy_callback(joy_msg)
    #     expected_input = {"x": -0.25, "y": 0.125, "theta": 0.6}
    #     self.assertEqual(self.omni_engine.joy_input, expected_input)
    #     print(f"Joy input after callback: {self.omni_engine.joy_input}")

    # def test_apply_joy_control(self):
    #     # Test joy input is sent to control_velocity
    #     mock_entity = MagicMock()
    #     mock_entity.id = 1
    #     mock_entity.__class__.__name__ = "Prey"
    #     self.omni_engine._entities = {mock_entity.id: mock_entity}
    #     self.omni_engine.control_velocity = MagicMock()

    #     self.omni_engine.apply_joy_control()
    #     self.omni_engine.control_velocity.assert_called_once_with(mock_entity.id, self.omni_engine.joy_input)

    # def test_control_velocity_mqtt_publish(self):
    #     # Test velocity control publishes correct message to MQTT
    #     entity_id = 1
    #     desired_velocity = {"x": 0.5, "y": 0.0, "theta": 1.0}
    #     self.omni_engine.control_velocity(entity_id, desired_velocity)
    #     expected_message = json.dumps(desired_velocity).encode('utf-8')
    #     self.mock_mqtt_client.publish.assert_called_once_with(
    #         f"/VSWARM{entity_id}_robot/motion", expected_message
    #     )

    # def test_update_led_color(self):
    #     # Test LED color update publishes correct message to MQTT
    #     mock_entity = MagicMock()
    #     mock_entity.id = 1
    #     mock_entity.color = 'blue'
    #     self.omni_engine._entities = {mock_entity.id: mock_entity}

    #     self.omni_engine.update_led_color()
    #     expected_led_color = 0x0000FF
    #     expected_json_msg_up = {
    #         "cmd_type": "ledup",
    #         "args_length": 6,
    #         "args": {
    #             "0": expected_led_color,
    #             "1": 14,
    #             "2": expected_led_color,
    #             "3": 14,
    #             "4": expected_led_color,
    #             "5": 14,
    #         },
    #     }
    #     expected_json_msg_down = {
    #         "cmd_type": "leddown",
    #         "args_length": 6,
    #         "args": {
    #             "0": expected_led_color,
    #             "1": 30,
    #             "2": expected_led_color,
    #             "3": 30,
    #             "4": expected_led_color,
    #             "5": 30,
    #         },
    #     }

    #     self.mock_mqtt_client.publish.assert_any_call(
    #         f"/VSWARM{mock_entity.id}_robot/cmd", json.dumps(expected_json_msg_up).encode('utf-8')
    #     )
    #     self.mock_mqtt_client.publish.assert_any_call(
    #         f"/VSWARM{mock_entity.id}_robot/cmd", json.dumps(expected_json_msg_down).encode('utf-8')
    #     )

    # def test_set_led_methods(self):
    #     # Test set_led* methods publish correct messages to MQTT
    #     entity_id = 1
    #     led_color = 0xFF0000
    #     self.omni_engine.set_ledup(entity_id, led_color)
    #     self.omni_engine.set_leddown(entity_id, led_color)

    #     expected_msg_up = json.dumps({
    #         "cmd_type": "ledup",
    #         "args_length": 6,
    #         "args": {"0": led_color, "1": 14, "2": led_color, "3": 14, "4": led_color, "5": 14}
    #     }).encode('utf-8')
    #     expected_msg_down = json.dumps({
    #         "cmd_type": "leddown",
    #         "args_length": 6,
    #         "args": {"0": led_color, "1": 30, "2": led_color, "3": 30, "4": led_color, "5": 30}
    #     }).encode('utf-8')

    #     self.mock_mqtt_client.publish.assert_any_call(
    #         f"/VSWARM{entity_id}_robot/cmd", expected_msg_up
    #     )
    #     self.mock_mqtt_client.publish.assert_any_call(
    #         f"/VSWARM{entity_id}_robot/cmd", expected_msg_down
    #     )


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist


class VelocityLimiterNode:
    def __init__(self):
        rospy.init_node('velocity_limiter_node', anonymous=True)

        # Parameters from roslaunch or default values
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.2)  # Default max linear speed
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # Publish rate in Hz
        self.time = rospy.get_param('~time', 15.0)
        self.vel_cmd_pub = rospy.Publisher('/robot/velcmd', Twist, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        self.timer = rospy.Timer(rospy.Duration(15.0), self.timer_callback)
        self.rate = rospy.Rate(self.publish_rate)

        rospy.spin()

    def cmd_vel_callback(self, msg):
        # Convert Twist message to numpy array
        linear_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        angular_vel = np.array([msg.angular.x, msg.angular.y, msg.angular.z])

        # Normalize linear velocity
        linear_norm = np.linalg.norm(linear_vel)
        if linear_norm > 0:
            linear_vel = linear_vel / linear_norm * self.max_linear_speed

        # Update Twist message with normalized and limited velocities
        msg.linear.x, msg.linear.y, msg.linear.z = linear_vel
        msg.angular.x, msg.angular.y, msg.angular.z = angular_vel

        # Publish limited velocity command
        self.vel_cmd_pub.publish(msg)

    def timer_callback(self, event):
        print(f"Closing node after {self.time} seconds timeout.")
        rospy.signal_shutdown("Timeout reached.")


if __name__ == '__main__':
    try:
        VelocityLimiterNode()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopJoy():
    def __init__(self):
        rospy.init_node("teleop_joy")

        # set parameter and it's default value
        rospy.set_param("max_linear_velocity", 0.3)
        rospy.set_param("max_angular_velocity", 1.5)

        self.velocity_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size = 10)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)

    def joy_callback(self, msg):
        # joystick values
        linear_speed = rospy.get_param("max_linear_velocity")
        angular_speed = rospy.get_param("max_angular_velocity")

        x = msg.axes[1] * linear_speed
        z = msg.axes[0] * angular_speed

        if x == 0 and z == 0:
            return

        twist = Twist()
        twist.linear.x = x
        twist.angular.z = z

        rospy.loginfo(f"Publishing linear.x: {x} angular.z: {z}")
        self.velocity_pub.publish(twist)

if __name__ == '__main__':
    try:
        teleopJoy = TeleopJoy()
        rospy.spin()

    except KeyboardInterrupt:
        pass

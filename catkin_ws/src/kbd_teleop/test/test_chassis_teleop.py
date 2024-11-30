#!/usr/bin/env python3

import unittest
import rospy
import rostest
from chassis_control.msg import SetVelocity

class TestKbdTeleop(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_kbd_teleop')
        self.received_msg = None
        self.sub = rospy.Subscriber('/chassis_control/set_velocity', 
                                  SetVelocity, self.callback)
        self.pub = rospy.Publisher('/chassis_control/set_velocity', 
                                 SetVelocity, queue_size=1)
        rospy.sleep(1)  # Wait for connections

    def callback(self, msg):
        self.received_msg = msg

    def test_forward_command(self):
        msg = SetVelocity()
        msg.velocity = 60.0
        msg.direction = 90
        msg.angular = 0.0
        self.pub.publish(msg)
        rospy.sleep(1)
        self.assertIsNotNone(self.received_msg)
        self.assertEqual(self.received_msg.velocity, 60.0)

    # Add more test methods as needed

if __name__ == '__main__':
    rostest.rosrun('kbd_teleop', 'test_kbd_teleop', TestKbdTeleop)
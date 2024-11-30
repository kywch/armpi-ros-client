#!/usr/bin/env python3
import unittest
import rospy
import rostest
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur

class TestArmServoControl(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_arm_servo_control')
        self.received_msg = None
        
        # Subscribe to the servo control topic
        self.sub = rospy.Subscriber(
            '/servo_controllers/port_id_1/multi_id_pos_dur',
            MultiRawIdPosDur,
            self.callback
        )
        
        # Publisher for sending test commands
        self.joints_pub = rospy.Publisher(
            '/servo_controllers/port_id_1/multi_id_pos_dur',
            MultiRawIdPosDur,
            queue_size=1
        )
        
        # Wait for publishers and subscribers to connect
        rospy.sleep(1)

    def callback(self, msg):
        """Store received message for verification"""
        self.received_msg = msg

    def test_servo_positions_first_sequence(self):
        """Test the first sequence of servo positions (350, 200)"""
        # Create and send test message
        test_positions = ((6, 350), (1, 200))
        duration = 1000
        
        msg = MultiRawIdPosDur(
            id_pos_dur_list=[
                RawIdPosDur(int(x[0]), int(x[1]), int(duration)) 
                for x in test_positions
            ]
        )
        
        self.joints_pub.publish(msg)
        rospy.sleep(0.5)  # Wait for message processing
        
        # Verify message was received
        self.assertIsNotNone(self.received_msg, "No message received")
        
        # Verify message content
        received_positions = [
            (pos.id, pos.position) 
            for pos in self.received_msg.id_pos_dur_list
        ]
        
        # Check each servo position
        for expected_id, expected_pos in test_positions:
            found = False
            for received_id, received_pos in received_positions:
                if received_id == expected_id:
                    self.assertEqual(received_pos, expected_pos, 
                        f"Servo {expected_id} position mismatch")
                    found = True
                    break
            self.assertTrue(found, f"Servo {expected_id} not found in response")

    def test_servo_positions_second_sequence(self):
        """Test the second sequence of servo positions (650, 500)"""
        # Create and send test message
        test_positions = ((6, 650), (1, 500))
        duration = 1000
        
        msg = MultiRawIdPosDur(
            id_pos_dur_list=[
                RawIdPosDur(int(x[0]), int(x[1]), int(duration)) 
                for x in test_positions
            ]
        )
        
        self.joints_pub.publish(msg)
        rospy.sleep(0.5)  # Wait for message processing
        
        # Verify message was received
        self.assertIsNotNone(self.received_msg, "No message received")
        
        # Verify message content
        received_positions = [
            (pos.id, pos.position) 
            for pos in self.received_msg.id_pos_dur_list
        ]
        
        # Check each servo position
        for expected_id, expected_pos in test_positions:
            found = False
            for received_id, received_pos in received_positions:
                if received_id == expected_id:
                    self.assertEqual(received_pos, expected_pos, 
                        f"Servo {expected_id} position mismatch")
                    found = True
                    break
            self.assertTrue(found, f"Servo {expected_id} not found in response")

    def test_duration_parameter(self):
        """Test that duration parameter is set correctly"""
        test_positions = ((6, 350), (1, 200))
        duration = 1000
        
        msg = MultiRawIdPosDur(
            id_pos_dur_list=[
                RawIdPosDur(int(x[0]), int(x[1]), int(duration)) 
                for x in test_positions
            ]
        )
        
        self.joints_pub.publish(msg)
        rospy.sleep(0.5)
        
        self.assertIsNotNone(self.received_msg, "No message received")
        
        # Verify duration for all servos
        for servo in self.received_msg.id_pos_dur_list:
            self.assertEqual(servo.duration, duration, 
                f"Duration mismatch for servo {servo.id}")

    def test_invalid_servo_id(self):
        """Test handling of invalid servo ID"""
        test_positions = ((99, 350),)  # Invalid servo ID
        duration = 1000
        
        msg = MultiRawIdPosDur(
            id_pos_dur_list=[
                RawIdPosDur(int(x[0]), int(x[1]), int(duration)) 
                for x in test_positions
            ]
        )
        
        self.joints_pub.publish(msg)
        rospy.sleep(0.5)
        
        # Add your error handling verification here
        # This will depend on how your system handles invalid servo IDs

if __name__ == '__main__':
    rostest.rosrun('kbd_teleop', 'test_arm_kbd_teleop', TestArmServoControl)
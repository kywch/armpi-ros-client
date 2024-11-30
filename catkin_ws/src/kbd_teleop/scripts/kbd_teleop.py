#!/usr/bin/env python3
# coding=utf8

import sys
import termios
import tty

import rospy
from chassis_control.msg import SetVelocity
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur

INITIAL_ARM_POSITIONS = ((6, 500), (5, 750), (4, 1000), (3, 350), (2, 500), (1, 300))
NEUTRAL_ARM_POSITIONS = ((6, 500), (5, 500), (4, 500), (3, 500), (2, 500), (1, 500))

class ChassisKeyboardTeleop:
    def __init__(self):
        rospy.init_node('chassis_keyboard_teleop', anonymous=True)
        self.pub_chassis = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
        self.pub_servos = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

        # Movement parameters
        self.linear_speed = 60.0  # Based on example code
        self.angular_speed = 0.3  # Based on example code
        self.direction_angle = 90  # 90 degrees for forward/backward
        
        # Set initial servo positions
        self.control_servos(INITIAL_ARM_POSITIONS)

        # Key mappings for chassis control - (linear_speed, direction_angle, angular_speed)
        self.chassis_key_bindings = {
            'w': (self.linear_speed, 90, 0.0),     # Forward
            's': (-self.linear_speed, 90, 0.0),    # Backward
            'a': (0.0, 90, self.angular_speed),    # Left turn (counterclockwise)
            'd': (0.0, 90, -self.angular_speed),   # Right turn (clockwise)
            ' ': (0.0, 0, 0.0),                    # Stop
        }

        self.servo_key_bindings = {
            '[': ((1, 0),),  # Open gripper
            ']': ((1, 1000),),  # Close gripper
            'p': INITIAL_ARM_POSITIONS,
            'o': NEUTRAL_ARM_POSITIONS,
        }

        # Store terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
    
    def getKey(self):
        """Get keyboard input."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def control_chassis(self, linear, direction, angular):
        """Publish movement command using SetVelocity message."""
        self.pub_chassis.publish(linear, direction, angular)

    def control_servos(self, pos_s, duration=1):
        """Publish servo control command using MultiRawIdPosDur message.
        
        Args:
            pos_s (tuple): A tuple of (servo_id, position) pairs.
            duration (int): Duration of the servo control in milliseconds.
        """
        msg = MultiRawIdPosDur(id_pos_dur_list=list(map(lambda x: RawIdPosDur(int(x[0]), int(x[1]), int(duration)), pos_s)))
        self.pub_servos.publish(msg)

    def run(self):
        """Main control loop."""
        print("Chassis Keyboard Teleop Active")
        print("------------------")
        print("w: forward")
        print("s: backward")
        print("a: left turn (counterclockwise)")
        print("d: right turn (clockwise)")
        print("space: stop")
        print("[: open gripper")
        print("]: close gripper")
        print("p: initial arm positions")
        print("o: neutral arm positions")
        print("q: quit")
        print("------------------")
        
        try:
            while not rospy.is_shutdown():
                key = self.getKey()
                
                if key == 'q':
                    break
                
                if key in self.chassis_key_bindings:
                    linear, direction, angular = self.chassis_key_bindings[key]
                    self.control_chassis(linear, direction, angular)
                elif key in self.servo_key_bindings:
                    pos_s = self.servo_key_bindings[key]
                    self.control_servos(pos_s)
                else:
                    # If an unknown key is pressed, stop the robot
                    self.control_chassis(0.0, 0.0, 0.0)
                    
        except Exception as e:
            print(e)
        
        finally:
            # Stop the robot before exiting
            self.control_chassis(0.0, 0.0, 0.0)
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    try:
        teleop = ChassisKeyboardTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
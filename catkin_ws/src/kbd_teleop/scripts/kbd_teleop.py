#!/usr/bin/env python3
# coding=utf8

import rospy
from chassis_control.msg import SetVelocity
import sys
import termios
import tty

class ChassisKeyboardTeleop:
    def __init__(self):
        rospy.init_node('chassis_keyboard_teleop', anonymous=True)
        self.publisher = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
        
        # Movement parameters
        self.linear_speed = 60.0  # Based on example code
        self.angular_speed = 0.3  # Based on example code
        self.direction_angle = 90  # 90 degrees for forward/backward
        
        # Key mappings - (linear_speed, direction_angle, angular_speed)
        self.key_bindings = {
            'w': (self.linear_speed, 90, 0.0),     # Forward
            's': (-self.linear_speed, 90, 0.0),    # Backward
            'a': (0.0, 90, self.angular_speed),    # Left turn (counterclockwise)
            'd': (0.0, 90, -self.angular_speed),   # Right turn (clockwise)
            ' ': (0.0, 0, 0.0),                    # Stop
        }
        
        # Store terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
    
    def getKey(self):
        """Get keyboard input."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_command(self, linear, direction, angular):
        """Publish movement command using SetVelocity message."""
        self.publisher.publish(linear, direction, angular)
    
    def run(self):
        """Main control loop."""
        print("Chassis Keyboard Teleop Active")
        print("------------------")
        print("w: forward")
        print("s: backward")
        print("a: left turn (counterclockwise)")
        print("d: right turn (clockwise)")
        print("space: stop")
        print("q: quit")
        print("------------------")
        
        try:
            while not rospy.is_shutdown():
                key = self.getKey()
                
                if key == 'q':
                    break
                
                if key in self.key_bindings:
                    linear, direction, angular = self.key_bindings[key]
                    self.publish_command(linear, direction, angular)
                else:
                    # If an unknown key is pressed, stop the robot
                    self.publish_command(0.0, 0.0, 0.0)
                    
        except Exception as e:
            print(e)
        
        finally:
            # Stop the robot before exiting
            self.publish_command(0.0, 0.0, 0.0)
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    try:
        teleop = ChassisKeyboardTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
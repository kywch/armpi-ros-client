#!/usr/bin/env python3
# coding=utf8

import sys
import termios
import tty

# For debugging
from pdb import set_trace as T

import os
os.environ['ROS_IP'] = '10.0.0.239'
os.environ['ROS_MASTER_URI'] = 'http://10.0.0.78:11311'
sys.path.append('catkin_ws/devel/lib/python3.8/site-packages')

import rospy
from chassis_control.msg import SetVelocity
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur
from armpi_ik import ArmpiKinematics


MIN_SERVO_STATE = 0
MAX_SERVO_STATE = 1000
ARM_MODES = {
    "navigate": ((6, 500), (5, 750), (4, 1000), (3, 350), (2, 120), (1, 1000)),
    "pickup": ((6, 500), (5, 200), (4, 850), (3, 730), (2, 500), (1, 0)),
    "handover": ((6, 500), (5, 500), (4, 500), (3, 500), (2, 120), (1, 1000)),
}


class Servo:
    def __init__(self, init_state, min_servo_state=MIN_SERVO_STATE, max_servo_state=MAX_SERVO_STATE):
        self.state = init_state
        self.min_servo_state = min_servo_state
        self.max_servo_state = max_servo_state

    def adjust(self, delta):
        if self.min_servo_state <= self.state + delta <= self.max_servo_state:
            self.state += delta
            return self.state

        # If not updated, return None
        return None


class ChassisKeyboardTeleop:
    def __init__(self):
        rospy.init_node('chassis_keyboard_teleop', anonymous=True)
        self.pub_chassis = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
        self.pub_servos = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.armpi = ArmpiKinematics()

        # Movement parameters
        self.linear_speed = 60.0  # Based on example code
        self.angular_speed = 0.3  # Based on example code
        self.direction_angle = 90  # 90 degrees for forward/backward

        self.arm_mode = None

        # Set initial servo positions
        self.set_arm_mode("navigate")

        self.servos = {
            6: Servo(init_state=500),  # base
            5: Servo(init_state=500, min_servo_state=200, max_servo_state=500),
            2: Servo(init_state=500, min_servo_state=120, max_servo_state=500),  # gripper roll
            1: Servo(init_state=1000),  # gripper
        }

        # Key mappings for chassis control - (linear_speed, direction_angle, angular_speed)
        self.chassis_key_bindings = {
            'w': (self.linear_speed, 90, 0.0),     # Forward
            's': (-self.linear_speed, 90, 0.0),    # Backward
            'a': (0.0, 90, self.angular_speed),    # Left turn (counterclockwise)
            'd': (0.0, 90, -self.angular_speed),   # Right turn (clockwise)
            ' ': (0.0, 0, 0.0),                    # Stop
        }

        self.servo_key_bindings = {
            "[": (self.adjust_arm_servo, {"servo_id": 1, "delta": -30}),
            "]": (self.adjust_arm_servo, {"servo_id": 1, "delta": 30}),
            "=": (self.toggle_gripper, {}),
            "u": (self.adjust_arm_servo, {"servo_id": 2, "delta": -10}),
            "o": (self.adjust_arm_servo, {"servo_id": 2, "delta": 10}),
            "i": (self.adjust_arm_servo, {"servo_id": 5, "delta": 10}),
            "k": (self.adjust_arm_servo, {"servo_id": 5, "delta": -10}),
            "j": (self.adjust_arm_servo, {"servo_id": 6, "delta": 10}),
            "l": (self.adjust_arm_servo, {"servo_id": 6, "delta": -10}),
            "1": (self.set_arm_mode, {"mode": "navigate"}),
            "2": (self.set_arm_mode, {"mode": "pickup"}),
            "3": (self.set_arm_mode, {"mode": "handover"}),
        }

        # Store terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
    
    def get_key(self):
        """Get keyboard input."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def control_chassis(self, linear, direction, angular):
        """Publish movement command using SetVelocity message."""
        self.pub_chassis.publish(linear, direction, angular)

    def control_servos(self, target_servo_states, duration=2):
        if target_servo_states is None:
            return

        msg = MultiRawIdPosDur(id_pos_dur_list=list(map(lambda x: RawIdPosDur(int(x[0]), int(x[1]), int(duration)), target_servo_states)))
        self.pub_servos.publish(msg)


    def set_arm_mode(self, mode):
        self.arm_mode = mode
        self.control_servos(ARM_MODES[self.arm_mode])
        return ARM_MODES[self.arm_mode]

    def adjust_arm_servo(self, servo_id, delta):
        assert servo_id in self.servos, "Invalid servo id"

        if self.arm_mode == "navigate" and servo_id == 5:
            return

        target = self.servos[servo_id].adjust(delta)
        if target is not None:
            self.control_servos([(servo_id, target)])

    def toggle_gripper(self):
        if self.arm_mode == "navigate":
            return
        
        if self.servos[1].state == MIN_SERVO_STATE:
            self.servos[1].state = MAX_SERVO_STATE
            self.control_servos([(1, MAX_SERVO_STATE)])
        else:
            self.servos[1].state = MIN_SERVO_STATE
            self.control_servos([(1, MIN_SERVO_STATE)])

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
                key = self.get_key()
                
                if key == 'q':
                    break
                
                if key in self.chassis_key_bindings:
                    linear, direction, angular = self.chassis_key_bindings[key]
                    self.control_chassis(linear, direction, angular)
                    rospy.sleep(0.1)

                elif key in self.servo_key_bindings:
                    func, args = self.servo_key_bindings[key]
                    target = func(**args)
                    self.control_servos(target)
                    rospy.sleep(0.05)

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
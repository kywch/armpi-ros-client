#!/usr/bin/env python3
# coding=utf8

"""Teleop using DualSense Wireless Controller"""

# # For debugging
# from pdb import set_trace as T
# import os
# import sys
# os.environ['ROS_IP'] = '10.0.0.239'  # The controller's IP address
# os.environ['ROS_MASTER_URI'] = 'http://10.0.0.78:11311'  # The robot's IP address
# sys.path.append('catkin_ws/devel/lib/python3.8/site-packages')

import rospy
import pygame

from chassis_control.msg import SetVelocity
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur
from ros_robot_controller.srv import GetBusServoState
from ros_robot_controller.msg import BusServoState, SetBusServoState, GetBusServoCmd

MIN_SERVO_STATE = 0
MAX_SERVO_STATE = 1000
ARM_MODES = {
    "navigate": ((6, 500), (5, 750), (4, 1000), (3, 350), (2, 120)),
    "pickup": ((6, 500), (5, 200), (4, 820), (3, 730), (2, 500)),
    "handover": ((6, 500), (5, 500), (4, 500), (3, 500), (2, 120)),
    "turnleft": ((6, 1000),),
    "turnright": ((6, 0),),
    "raise": ((5, 500),),
    "lower": ((5, 200),),
}

def servo_stop_msg(servo_id):
    msg = BusServoState()
    msg.present_id = [1, servo_id]
    msg.stop = [1]
    return msg

STOP_ARM_SERVOS = [servo_stop_msg(i) for i in range(2, 7)]


class ArmpiJoystickTeleop:
    def __init__(self, debug=False):
        rospy.init_node('dsw_teleop', anonymous=True)
        self.chassis_pub = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
        self.arm_servo_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

        # NOTE: ros_robot_controller has more functions, but here we use this to stop the arm servos.
        self.rrc_set_pub = rospy.Publisher('/ros_robot_controller/bus_servo/set_state', SetBusServoState, queue_size=10)
        self.rrc_get_proxy = rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state', GetBusServoState)

        # Setup joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() < 1:
            raise RuntimeError("No controller found")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # Set the speed of arm movement. Make it slower for debugging.
        self.arm_duration = 3 if debug else 1

        # Set initial servo positions
        self.gripper_state = None
        self.toggle_gripper(open=True)
        self.control_servos(ARM_MODES["navigate"])

        # Movement params and key mappings for chassis control
        self.linear_speed = 150.0  # Based on example code
        self.angular_speed = 0.3  # Based on example code
        self.direction_angle = 90  # 90 degrees for forward/backward

        self.arm_servo_bindings = {
            0: {"msg": "X pressed. Stop Moving.", "func": self.stop_moving, "args": ()},
            2: {"msg": "△ pressed. Set arm to navigate.", "func": self.control_servos, "args": (ARM_MODES["navigate"], 2)},
            6: {"msg": "L2 pressed. Set arm to pickup.", "func": self.control_servos, "args": (ARM_MODES["pickup"], 2)},
            4: {"msg": "L1 pressed. Set arm to handover.", "func": self.control_servos, "args": (ARM_MODES["handover"], 2)},
            3: {"msg": "□ pressed. Turn left.", "func": self.control_servos, "args": (ARM_MODES["turnleft"], 3)},
            1: {"msg": "o pressed. Turn right.", "func": self.control_servos, "args": (ARM_MODES["turnright"], 3)},
            5: {"msg": "R1 pressed. Raise arm.", "func": self.pivot_arm_if_possible, "args": ("raise",)},
            7: {"msg": "R2 pressed. Lower arm.", "func": self.pivot_arm_if_possible, "args": ("lower",)},
            9: {"msg": "☰ pressed. Toggle gripper.", "func": self.toggle_gripper, "args": ()},
        }

        self.quit_key_bindings = {
            10: {"msg": "PS pressed. Exiting..."},
        }

    def stop_moving(self):
        # Stop the chassis
        self.control_chassis(0, 0, 0)
        # Stop the arm servos
        msg = SetBusServoState()
        msg.state = STOP_ARM_SERVOS
        msg.duration = 0
        self.rrc_set_pub.publish(msg)
        rospy.sleep(0.1)

    def toggle_gripper(self, open=False):
        if open or self.gripper_state is None:
            self.gripper_state = MIN_SERVO_STATE
        elif self.gripper_state == MIN_SERVO_STATE:
            self.gripper_state = MAX_SERVO_STATE
        else:
            self.gripper_state = MIN_SERVO_STATE
        self.control_servos(((1, self.gripper_state),))
        rospy.sleep(0.1)

    def control_chassis(self, linear, direction, angular):
        """Publish movement command using SetVelocity message."""
        self.chassis_pub.publish(linear, direction, angular)

    def control_servos(self, pos_s, duration=None):
        """Publish servo control command using MultiRawIdPosDur message.
        
        Args:
            pos_s (tuple): A tuple of (servo_id, position) pairs.
            duration (int): Duration of the servo control in milliseconds.
        """
        duration = duration or self.arm_duration
        msg = MultiRawIdPosDur(id_pos_dur_list=list(map(lambda x: RawIdPosDur(int(x[0]), int(x[1]), int(duration)), pos_s)))
        self.arm_servo_pub.publish(msg)
        rospy.sleep(0.1)

    def pivot_arm_if_possible(self, mode, duration=3):
        assert mode in ["raise", "lower"], "Invalid mode"
        
        # Get current the position of servo 5
        msg = GetBusServoCmd()
        msg.id = 5  # Servo ID
        msg.get_position = 1
        res = self.rrc_get_proxy([msg]).state[0]
        if res.position[0] > 550:  # raise/lower arm with servo 5 is done between 200 and 500, plus some error margin
            print("Cannot pivot arm")
            return

        self.control_servos(ARM_MODES[mode], duration)
        rospy.sleep(0.1)

    def close(self):
        self.stop_moving()
        pygame.joystick.quit()
        pygame.quit()

    def run(self):
        """Main control loop."""
        print("DSW Teleop Active")
        print("------------------")
        print("Press the PS key to exit.")
        print("------------------")

        running = True
        try:
            while running and not rospy.is_shutdown():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                        break

                    # Button presses
                    if event.type == pygame.JOYBUTTONDOWN:
                        if event.button in self.quit_key_bindings:
                            print(self.quit_key_bindings[event.button]["msg"])
                            running = False
                            break

                        elif event.button in self.arm_servo_bindings:
                            # For arm movement, stop the chassis first
                            self.control_chassis(0, 0, 0)
                            rospy.sleep(0.1)

                            # Then call the arm movement function
                            binding = self.arm_servo_bindings[event.button]
                            print(binding["msg"])
                            binding["func"](*binding["args"])

                    # D-pad movement
                    elif event.type == pygame.JOYHATMOTION:
                        x, y = event.value
                        angular = -self.angular_speed * x
                        linear = self.linear_speed * y
                        if y == 1:
                            print("D-pad Up pressed")
                        elif y == -1:
                            print("D-pad Down pressed")
                        elif x == -1:
                            print("D-pad Left pressed")
                        elif x == 1:
                            print("D-pad Right pressed")
                        self.control_chassis(linear, self.direction_angle, angular)
                        rospy.sleep(0.1)

        except Exception as e:
            print(e)

        finally:
            # Stop the robot before exiting
            self.close()

if __name__ == '__main__':
    try:
        teleop = ArmpiJoystickTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
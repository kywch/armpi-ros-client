#!/usr/bin/env python3

import numpy as np
import tinyik

MIN_SERVO_STATE = 0
MAX_SERVO_STATE = 1000
INIT_SERVO_STATE = 500
DEG_120_RAD = np.pi / 3
DEG_90_RAD = np.pi / 4

EE_PITCH_SERVO_ID = 3
EE_ROLL_SERVO_ID = 2
GRIPPER_SERVO_ID = 1


class Servo:
    """A simple class to convert radians to servo states and vice versa"""
    def __init__(self,
                 min_radian,
                 max_radian,
                 min_servo_state=MIN_SERVO_STATE,
                 max_servo_state=MAX_SERVO_STATE,
    ):
        self.min_servo_state = min_servo_state
        self.max_servo_state = max_servo_state
        self.init_servo_state = (self.max_servo_state + self.min_servo_state) / 2

        self.min_radian = min_radian
        self.max_radian = max_radian
        self.radian_to_servo_scale = (self.max_servo_state - self.min_servo_state) / (self.max_radian - self.min_radian)

    def radian_to_servo(self, radian):
        return int(round(self.init_servo_state + radian * self.radian_to_servo_scale))

    def servo_to_radian(self, servo_state):
        return (servo_state - self.init_servo_state) / self.radian_to_servo_scale


class ArmpiKinematics:
    # Define servo ranges for each joint (pulse width range, angle range)
    # NOTE: angles are in radians
    def __init__(self):
        # 4-dof arm without the gripper, from Servo 6, Servo 5, Servo 4, Servo 3
        # [0, pi/4, 0, 0] is the reset position, with the gripper end at [0, 6, 55]
        self._actuator = tinyik.Actuator(
            ["z", [0, 0, 0], "x", [0, 0, 10.16], "x", [0, 0, 9.64], "x", [0, 6, 18.5]]
        )

        assert np.array_equal(self._actuator.angles, [0, 0, 0, 0])
        assert np.allclose(self._actuator.ee, [0, 6, 38.3])

        self._max_reach = np.linalg.norm(self._actuator.ee)
        self._gripper_servo_state = MAX_SERVO_STATE  # Closed
        self._gripper_roll_servo_state = INIT_SERVO_STATE

        # Always store the current angles, so that we can use tinyik to check the validity of new actions
        self._actuator_angle_buffer = self._actuator.angles.copy()

        self.servo_to_angle_idx = { 6: 0, 5: 1, 4: 2, EE_PITCH_SERVO_ID: 3 }
        self.servos = {
            6: Servo(-DEG_120_RAD, DEG_120_RAD),
            5: Servo(-DEG_90_RAD, DEG_90_RAD),
            4: Servo(-DEG_120_RAD, DEG_120_RAD),
            EE_PITCH_SERVO_ID: Servo(-DEG_120_RAD, DEG_120_RAD),
            EE_ROLL_SERVO_ID: Servo(-DEG_120_RAD, DEG_120_RAD),
        }

    def reset(self):
        self._actuator_angle_buffer[:] = self._actuator.angles = [0, 0, 0, 0]
        self._gripper_servo_state = MAX_SERVO_STATE  # Closed
        self._gripper_roll_servo_state = INIT_SERVO_STATE
        return [(k, v) for k, v in self.servo_states.items()]

    @property
    def servo_states(self):
        return {
            6: self.servos[6].radian_to_servo(self._actuator.angles[0]),  # Base servo
            5: self.servos[5].radian_to_servo(self._actuator.angles[1]),
            4: self.servos[4].radian_to_servo(self._actuator.angles[2]),
            EE_PITCH_SERVO_ID: self.servos[3].radian_to_servo(self._actuator.angles[3]),
            EE_ROLL_SERVO_ID: self._gripper_roll_servo_state,
            GRIPPER_SERVO_ID: self._gripper_servo_state  # Gripper open/close
        }

    @property
    def ee_pos(self):
        return self._actuator.ee

    def set_servo_states(self, servo_states):
        for k, v in servo_states:
            assert k in [1, 2, 3, 4, 5, 6], "Invalid servo ID"
            assert v >= MIN_SERVO_STATE and v <= MAX_SERVO_STATE, "Invalid servo state"

        new_angles = self._actuator.angles.copy()
        for k, v in servo_states:
            if k == 1:
                self._gripper_servo_state = v
            elif k == 2:
                self._gripper_roll_servo_state = v
            else:
                new_angles[self.servo_to_angle_idx[k]] = self.servos[k].servo_to_radian(v)

        self._actuator.angles = new_angles

        # Return the new ee_pos
        return self.ee_pos

    # APIs for the teleop
    def adjust_ee_pos_by(self, delta_x, delta_y, delta_z):
        self._actuator_angle_buffer[:] = self._actuator.angles
        
        # Check if the new ee_pos is valid, and if not valid, return None and do nothing
        if self.ee_pos[2] + delta_z < -15:  # Cannot go below the ground (the base is 15cm above the ground)
            print("Cannot go below the ground")
            return None

        new_ee_pos = self.ee_pos + np.array([delta_x, delta_y, delta_z])
        if np.linalg.norm(new_ee_pos) > self._max_reach:  # Cannot go beyond the max reach
            print("Cannot go beyond the max reach")
            return None

        # Update the actuator and see if the servo angles are valid
        self._actuator.ee = new_ee_pos  # this updates the angles
        target_dict = {
            6: self.servos[6].radian_to_servo(self._actuator.angles[0]),  # Base servo
            5: self.servos[5].radian_to_servo(self._actuator.angles[1]),
            4: self.servos[4].radian_to_servo(self._actuator.angles[2]),
            EE_PITCH_SERVO_ID: self.servos[EE_PITCH_SERVO_ID].radian_to_servo(self._actuator.angles[3]),
        }

        for v in target_dict.values():
            if v < MIN_SERVO_STATE or v > MAX_SERVO_STATE:
                print("New servo state is invalid")
                print(target_dict)
                # revert the actuator angles
                self._actuator.angles = self._actuator_angle_buffer
                return None

        # Return the target servo states
        return [(k, v) for k, v in target_dict.items()]

    def adjust_ee_pitch_by(self, delta_pitch):
        # Check if the delta_pitch is valid
        pitch_servo_state = self.servos[EE_PITCH_SERVO_ID].radian_to_servo(self._actuator.angles[EE_PITCH_SERVO_ID])
        pitch_servo_state += delta_pitch

        # If not valid, do nothing, return None
        if pitch_servo_state < MIN_SERVO_STATE or pitch_servo_state > MAX_SERVO_STATE:
            return None
        
        # Valid change: Update and return the target servo states
        self._actuator.angles[EE_PITCH_SERVO_ID] = self.servos[EE_PITCH_SERVO_ID].servo_to_radian(pitch_servo_state)

        return ((EE_PITCH_SERVO_ID, pitch_servo_state),)

    def adjust_ee_roll_by(self, delta_roll):
        # Check if the delta_roll is valid
        roll_servo_state = self.servos[EE_ROLL_SERVO_ID].radian_to_servo(self._actuator.angles[EE_ROLL_SERVO_ID])
        roll_servo_state += delta_roll

        # If not valid, do nothing, return None
        if roll_servo_state < MIN_SERVO_STATE or roll_servo_state > MAX_SERVO_STATE:
            return None

        # Valid change: Update and return the target servo states
        self._actuator.angles[EE_ROLL_SERVO_ID] = self.servos[EE_ROLL_SERVO_ID].servo_to_radian(roll_servo_state)

        return ((EE_ROLL_SERVO_ID, roll_servo_state),)

    def set_gripper(self, action):
        assert action in ["open", "close"], "Invalid gripper action"
        if action == "open":
            self._gripper_servo_state = MIN_SERVO_STATE
        elif action == "close":
            self._gripper_servo_state = MAX_SERVO_STATE
        return ((GRIPPER_SERVO_ID, self._gripper_servo_state),)

    def toggle_gripper(self):
        if self._gripper_servo_state == MIN_SERVO_STATE:
            self._gripper_servo_state = MAX_SERVO_STATE
        else:
            self._gripper_servo_state = MIN_SERVO_STATE
        return ((GRIPPER_SERVO_ID, self._gripper_servo_state),)


if __name__ == "__main__":
    # For debugging
    from pdb import set_trace as T

    import os
    import sys
    os.environ['ROS_IP'] = '10.0.0.239'
    os.environ['ROS_MASTER_URI'] = 'http://10.0.0.78:11311'
    sys.path.append('catkin_ws/devel/lib/python3.8/site-packages')

    import rospy
    from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur

    # Set duration > 1 to make the movement slower, for testing
    def control_servos(pub_servos, target_servo_states, duration=3):
        """Publish servo control command using MultiRawIdPosDur message.
        
        Args:
            pos_s (tuple): A tuple of (servo_id, position) pairs.
            duration (int): Duration of the servo control in milliseconds.
        """
        if target_servo_states is None:
            return

        msg = MultiRawIdPosDur(id_pos_dur_list=list(map(lambda x: RawIdPosDur(int(x[0]), int(x[1]), int(duration)), target_servo_states)))
        pub_servos.publish(msg)

    # Initialize ROS node for testing
    rospy.init_node('ik_test', log_level=rospy.DEBUG)
    
    # Create publisher for servo control
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', 
                                 MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2)

    armpi = ArmpiKinematics()

    target = [(k, v) for k, v in armpi.servo_states.items()]

    control_servos(joints_pub, target)

    target = armpi.toggle_gripper()
    control_servos(joints_pub, target)

    target = armpi.set_gripper("open")
    control_servos(joints_pub, target)

    target = armpi.adjust_ee_pitch_by(50)
    control_servos(joints_pub, target)

    target = armpi.adjust_ee_pos_by(0, 0, -8)
    control_servos(joints_pub, target)
    print(armpi.ee_pos, armpi._actuator.angles)

    rospy.sleep(5)

    target = armpi.adjust_ee_pos_by(0, 20, 0)
    control_servos(joints_pub, target)
    print(armpi.ee_pos, armpi._actuator.angles)

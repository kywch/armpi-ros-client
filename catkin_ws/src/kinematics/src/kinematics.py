#!/usr/bin/env python3
# encoding: utf-8
# 4-DOF Robot Arm Kinematics: Calculate joint angles (IK) and end effector position (FK)

import logging
from math import degrees, atan, sqrt, sin, cos, atan2, acos, radians

# CRITICAL, ERROR, WARNING, INFO, DEBUG
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class Kinematics:
    # Servos counted from bottom to top
    # Common parameters - link parameters for 4-DOF robotic arm
    l1 = 6.10    # Distance from base center to second servo axis center (6.10cm)
    l2 = 10.16   # Distance between second and third servo (10.16cm)
    l3 = 9.64    # Distance between third and fourth servo (9.64cm)
    l4 = 0.00    # Not assigned here, will be set based on initialization choice
    
    # Parameters specific to pump model
    l5 = 4.70    # Distance from fourth servo to directly above suction nozzle (4.70cm)
    l6 = 4.46    # Distance from above suction nozzle to nozzle tip (4.46cm)
    alpha = degrees(atan(l6 / l5))  # Calculate angle between l5 and l4

    def __init__(self, arm_type='arm'): # Adapt parameters based on different end effector types
        self.arm_type = arm_type
        if self.arm_type == 'pump': # If it's a pump-type arm
            self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))  # Fourth link length is from servo to suction nozzle
        elif self.arm_type == 'arm':
            self.l4 = 16.65  # Distance from fourth servo to arm end (16.6cm), measured with gripper fully closed

    def set_link_length(self, L1=l1, L2=l2, L3=l3, L4=l4, L5=l5, L6=l6):
        # Modify arm link lengths to accommodate similar structures with different dimensions
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3
        self.l4 = L4
        self.l5 = L5
        self.l6 = L6
        if self.arm_type == 'pump':
            self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))
            self.alpha = degrees(atan(self.l6 / self.l5))

    def get_link_length(self):
        # Get current link length settings
        if self.arm_type == 'pump':
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4, "L5":self.l5, "L6":self.l6}
        else:
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4}

    def get_rotation_angle(self, coordinate_data, Alpha):
        # Calculate joint rotation angles given target coordinates and pitch angle
        # coordinate_data is end effector coordinates in cm, passed as tuple, e.g. (0, 5, 10)
        # Alpha is the angle between end effector and horizontal plane, in degrees
        X, Y, Z = coordinate_data
        if self.arm_type == 'pump':
            Alpha -= self.alpha

        # Calculate base rotation angle
        theta6 = degrees(atan2(Y, X))
 
        P_O = sqrt(X*X + Y*Y) # Distance from P_ to origin O
        CD = self.l4 * cos(radians(Alpha))
        PD = self.l4 * sin(radians(Alpha)) # PD positive when pitch angle positive, negative when pitch angle negative
        AF = P_O - CD
        CF = Z - self.l1 - PD
        AC = sqrt(pow(AF, 2) + pow(CF, 2))

        if round(CF, 4) < -self.l1:
            logger.debug('Height below 0, CF(%s)<l1(%s)', CF, -self.l1)
            return False
        if self.l2 + self.l3 < round(AC, 4): # Sum of two sides less than third side
            logger.debug('Cannot form linkage structure, l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
            return False

        # Calculate theta4
        cos_ABC = round(-(pow(AC, 2)- pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*self.l3), 4) # Cosine theorem
        if abs(cos_ABC) > 1:
            logger.debug('Cannot form linkage structure, abs(cos_ABC(%s)) > 1', cos_ABC)
            return False
        ABC = acos(cos_ABC) # Convert to radians using inverse cosine
        theta4 = 180.0 - degrees(ABC)

        # Calculate theta5
        CAF = acos(AF / AC)
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*AC), 4) # Cosine theorem
        if abs(cos_BAC) > 1:
            logger.debug('Cannot form linkage structure, abs(cos_BAC(%s)) > 1', cos_BAC)
            return False
        if CF < 0:
            zf_flag = -1
        else:
            zf_flag = 1
        theta5 = degrees(CAF * zf_flag + acos(cos_BAC))

        # Calculate theta3
        theta3 = Alpha - theta5 + theta4
        if self.arm_type == 'pump':
            theta3 += self.alpha
        return {"theta3":theta3, "theta4":theta4, "theta5":theta5, "theta6":theta6}

    def get_coordinate(self, angle_data):
        """Calculate end effector position given joint angles
        
        Args:
            angle_data (dict): Joint angles in degrees with keys "theta3", "theta4", "theta5", "theta6"
        
        Returns:
            tuple: (X, Y, Z) coordinates in cm and pitch angle Alpha in degrees
            False: If invalid angles provided
        """
        theta3 = angle_data["theta3"]
        theta4 = angle_data["theta4"] 
        theta5 = angle_data["theta5"]
        theta6 = angle_data["theta6"]

        # Adjust theta3 for pump type
        if self.arm_type == 'pump':
            theta3 -= self.alpha

        # Convert angles to radians for calculations
        t3_rad = radians(theta3)
        t4_rad = radians(theta4)
        t5_rad = radians(theta5)
        t6_rad = radians(theta6)

        # Calculate pitch angle Alpha
        Alpha = theta3 + theta5 - theta4
        if self.arm_type == 'pump':
            Alpha += self.alpha

        # Calculate arm positions step by step
        # Start with base height
        Z = self.l1

        # Add second link contribution (l2)
        Z += self.l2 * sin(t5_rad)
        X_r = self.l2 * cos(t5_rad)  # Radial distance from base

        # Add third link contribution (l3)
        Z += self.l3 * sin(t5_rad - t4_rad)
        X_r += self.l3 * cos(t5_rad - t4_rad)

        # Add fourth link contribution (l4)
        Z += self.l4 * sin(radians(Alpha))
        X_r += self.l4 * cos(radians(Alpha))

        # Convert radial distance to X,Y coordinates
        X = X_r * cos(t6_rad)
        Y = X_r * sin(t6_rad)

        return (X, Y, Z, Alpha)

if __name__ == '__main__':
    ik = Kinematics('arm')
    ik.set_link_length(L1=ik.l1 + 0.89, L4=ik.l4 - 0.3)
    print('Link lengths:', ik.get_link_length())
    
    # Test IK
    coord = (0, 0, ik.l1 + ik.l2 + ik.l3 + ik.l4)
    angles = ik.get_rotation_angle(coord, 90)
    print("IK Test:")
    print(f"Input coordinates: {coord}, pitch: 90")
    print(f"Joint angles: {angles}")
    
    # Test FK
    if angles:
        result = ik.get_coordinate(angles)
        print("\nFK Test:")
        print(f"Input angles: {angles}")
        print(f"Output coordinates: ({result[0]:.2f}, {result[1]:.2f}, {result[2]:.2f})")
        print(f"Output pitch angle: {result[3]:.2f}")
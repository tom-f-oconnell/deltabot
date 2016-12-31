#!/usr/bin/env python3

import numpy as np

# mostly following terminology in team "Delta Force" senior project, from Columbia
# (x,y) = (0,0) is defined as the center of the base the motors are symmetrically
# attached to

# the motor are separated by an angle alpha
alpha = 2 * np.pi / 3.0 # radians
alpha_offset = np.pi / 2.0

# radius from center of base to x, y position of motor joint
rm = 10 # mm
upper_arm_length = 20 # mm

# motor rotor angles (radians), from x-y plane
# motor A will be defined as at pi degrees
# (its upper arm will rotate in a plane parallel to Y axis, and orthogonal to Y and X)
thetaA = 0
thetaB = 0
thetaC = 0

# TODO draw out. visualize?
# motor joint = "shoulder"
# they call the upper joint the "elbow"
# an the lower joint the "wrist"
elbow_x_A = (rm + upper_arm_length * np.cos(thetaA)) * np.cos(alpha_offset)
elbow_x_B = (rm + upper_arm_length * np.cos(thetaB)) * np.cos(alpha_offset + alpha)
elbow_x_C = (rm + upper_arm_length * np.cos(thetaC)) * np.cos(alpha_offset + 2 * alpha)

elbow_y_A = (rm + upper_arm_length * np.cos(thetaA)) * np.sin(alpha_offset)
elbow_y_B = (rm + upper_arm_length * np.cos(thetaB)) * np.sin(alpha_offset + alpha)
elbow_y_C = (rm + upper_arm_length * np.cos(thetaC)) * np.sin(alpha_offset + 2 * alpha)

elbow_z_A = upper_arm_length * np.sin(thetaA)
elbow_z_B = upper_arm_length * np.sin(thetaB)
elbow_z_C = upper_arm_length * np.sin(thetaC)

# 'the wrist must lie on a sphere centered at the elbow with radius equal to the length of the 
#  lower arm'
# all joints besides the motor <-> upper arm are double ball and sockets
# there will (always?) be two real intersections of these three spheres
# and the intersection below the motor mount is the one of interest

lower_arm_length = 100 # mm

# Cartesian coordinates of end effector's center
effector_cx = 
effector_cy = 
effector_cz = 

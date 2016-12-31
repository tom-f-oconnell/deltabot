#!/usr/bin/env python3

import numpy as np

# mostly following terminology in team "Delta Force" senior project, from Columbia
# (x,y) = (0,0) is defined as the center of the base the motors are symmetrically
# attached to

# the motor are separated by an angle alpha
alpha = 2 * np.pi / 3.0 # radians
alpha_offset = np.pi / 2.0

# radius from center of base to x, y position of motor joint
rm = 50 # mm
upper_arm_length = 40 # mm

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

lower_arm_length = 170 # mm

# Cartesian coordinates of end effector's center
# using equations described on the Wikipedia article for trilateration
# see also page 16-17 of my blue notebook from Michael's rotation

# the centers of our spheres
a = np.array((elbow_x_A, elbow_y_A, elbow_z_A))
b = np.array((elbow_x_B, elbow_y_B, elbow_z_B))
c = np.array((elbow_x_C, elbow_y_C, elbow_z_C))

# first put the sphere centers in a coordinate system with all 3 coplanar in z=0 (x-y)
# one at the x-y origin, and one along the x-axis

# make the new (unit) basis vectors (e_)
dx = b - a
ex = dx / np.linalg.norm(dx)
i = np.dot(ex, c - a)
dy = c - a - i*ex
ey = dy / np.linalg.norm(dy)
ez = np.cross(ex, ey)

# quantities in the diagram on the Wikipedia page
d = np.linalg.norm(b - a)
j = np.dot(ey, c - a)

# for convenience
r = lower_arm_length

# the assumptions guaranteeing non-zero, finite intersections
assert d < 2*r, 'spheres are disjoint'
assert d > 0, 'spheres have same origin'

# arm length (sphere radius) actually cancels out of most of these 
# because all of r1,r2,and r3 and the same
x = d**2 / (2*d)
y = (i**2 + j**2 + x*i) / (2*j)

discriminant = r**2 - x**2 - y**2
# TODO is this always right? safe after above check?
assert discriminant >= 0, 'no real roots => spheres seem not to intersect'
z = np.sqrt(discriminant)

# position of the center of the end effector (?) (assuming symmetric)
# in the original coordinate system
# TODO not assuming all ball joints overlap, is it?
effector = a + x*ex + y*ey + z*ez
print(effector)

# TODO is it really only possible for end effector to be parallel to base (and thus ground)?
# if not, visualize z differences

# TODO TODO TODO why are x and y different distance from origin (which should be center of
# motor mount), despite supposed symmetry about motor base center?


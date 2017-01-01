#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

verbose = False
should_plot = False

sns.set_style('dark')

# mostly following terminology in team "Delta Force" senior project, from Columbia
# (x,y) = (0,0) is defined as the center of the base the motors are symmetrically
# attached to

def forward_kinematics(thetaA, thetaB, thetaC, alpha=None, alpha_offset=None, \
        rm=50, upper_arm_length=40, lower_arm_length=170):
    """ Takes 3 motor angles and outputs the Cartesian coordinates of the end effector,
        with the point between the three motors as the origin.

        Parameters:
        alpha (radians) - angular separation of motors (default: 2/3 pi = 120 deg)
                don't currently support other values
        alpha_offset (radians) - angular placement of first motor
        rm (mm) - radius from center of base to x, y position of motor joint
        upper_arm_length (mm) - connecting motor to lower arm
        lower_arm_length (mm) - connecting upper arm to effector """

    if alpha == None:
        alpha = 2 * np.pi / 3.0 # radians
    if alpha_offset == None:
        alpha_offset = np.pi / 2.0

    # TODO draw out. visualize? TODO
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

    # Cartesian coordinates of end effector's center
    # using equations described on the Wikipedia article for trilateration
    # see also page 16-17 of my blue notebook from Michael's rotation

    # the centers of our spheres
    a = np.array((elbow_x_A, elbow_y_A, elbow_z_A))
    b = np.array((elbow_x_B, elbow_y_B, elbow_z_B))
    c = np.array((elbow_x_C, elbow_y_C, elbow_z_C))

    # check that, in the case when motor angles are equal, elbows are equidistant from central origin
    if thetaA == thetaB:
        assert np.isclose(np.linalg.norm(a), np.linalg.norm(b)), 'equal effector angles ' + \
                'should lead to elbows equidistant from the origin, at the center of the 3 motors'
    if thetaA == thetaC:
        assert np.isclose(np.linalg.norm(a), np.linalg.norm(c)), 'equal effector angles ' + \
                'should lead to elbows equidistant from the origin, at the center of the 3 motors'
    if thetaB == thetaC:
        assert np.isclose(np.linalg.norm(b), np.linalg.norm(c)), 'equal effector angles ' + \
                'should lead to elbows equidistant from the origin, at the center of the 3 motors'

    if verbose:
        print('Elbow positions:')
        print(a)
        print(b)
        print(c)
        print('')

    if should_plot:
        plt.figure()
        plt.plot(a[0], a[1], '.', label='a')
        plt.plot(b[0], b[1], '.', label='b')
        plt.plot(c[0], c[1], '.', label='c')

    # first put the sphere centers in a coordinate system with all 3 coplanar in z=0 (x-y)
    # one at the x-y origin, and one along the x-axis

    # make the new (unit) basis vectors (e_)
    dx = b - a
    ex = dx / np.linalg.norm(dx)
    i = np.dot(ex, c - a)
    dy = c - a - i*ex
    ey = dy / np.linalg.norm(dy)
    ez = np.cross(ex, ey)

    # check the basis is orthonormal
    assert np.isclose(np.dot(ex,ey), 0), 'basis vectors not orthogonal'
    assert np.isclose(np.dot(ey,ez), 0), 'basis vectors not orthogonal'
    assert np.isclose(np.dot(ex,ez), 0), 'basis vectors not orthogonal'

    assert np.isclose(np.linalg.norm(ex), 1), 'basis vectors not unit vectors'
    assert np.isclose(np.linalg.norm(ey), 1), 'basis vectors not unit vectors'
    assert np.isclose(np.linalg.norm(ez), 1), 'basis vectors not unit vectors'

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
    y = (i**2 + j**2 - 2*x*i) / (2*j)

    discriminant = r**2 - x**2 - y**2
    # TODO is this always right? safe after above check?
    assert discriminant >= 0, 'no real roots => spheres seem not to intersect'
    z = np.sqrt(discriminant)

    show_temp_basis = False

    # show some of the intermediate calculations, to get an intuititive geometric sense
    # for what might be going wrong
    if show_temp_basis:
        ax = plt.gca()
        arrow_scale = 40
        # TODO labels dont seem to work on these
        ax.arrow(0, 0, ex[0]*arrow_scale, ex[1]*arrow_scale, \
                label='ex', color='r', width=arrow_scale*0.005)

        ax.arrow(0, 0, ey[0]*arrow_scale, ey[1]*arrow_scale, \
                label='ey', color='g', width=arrow_scale*0.005)

        # basically has a zero projection on plotted plane, as expected
        ax.arrow(0, 0, ez[0]*arrow_scale, ez[1]*arrow_scale, \
                label='ez', color='b', width=arrow_scale*0.005)

    # position of the center of the end effector (?) (assuming symmetric)
    # in the original coordinate system
    # we want the solution with a negative z, presumably
    # TODO not assuming all ball joints overlap, is it?
    effector = a + x*ex + y*ey - z*ez

    if verbose:
        print('End effector center:', effector)
        print('')

    if should_plot:
        plt.plot(effector[0], effector[1], 'r.', label='effector')
        plt.legend()
        plt.show()

    return effector

# TODO is it really only possible for end effector to be parallel to base (and thus ground)?
# if not, visualize z differences

# motor rotor angles (radians), from x-y plane
thetaA = (2*np.pi / 360) * -10
thetaB = (2*np.pi / 360) * -10
thetaC = (2*np.pi / 360) * -10

# calculate the end effector position
print(forward_kinematics(thetaA, thetaB, thetaC))

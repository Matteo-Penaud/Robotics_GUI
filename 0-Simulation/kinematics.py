import math
import time
from numpy.lib.twodim_base import tri
from constants import *
from scipy.optimize import minimize
import numpy as np

# Given the sizes (a, b, c) of the 3 sides of a triangle, returns the angle between a and b using the alKashi theorem.
def alKashi(a, b, c, sign=-1):
    if a * b == 0:
        print("WARNING a or b is null in AlKashi")
        return 0
    # Note : to get the other altenative, simply change the sign of the return :
    return sign * math.acos(min(1, max(-1, (a ** 2 + b ** 2 - c ** 2) / (2 * a * b))))


# Computes the direct kinematics of a leg in the leg's frame
# Given the angles (theta1, theta2, theta3) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the destination point (x, y, z)
def computeDK(
    theta1,
    theta2,
    theta3,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    use_rads=USE_RADS_INPUT,
    use_mm=USE_MM_OUTPUT,
):
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit
    # print(
    #     "corrected angles={}, {}, {}".format(
    #         theta1 * (1.0 / angle_unit),
    #         theta2 * (1.0 / angle_unit),
    #         theta3 * (1.0 / angle_unit),
    #     )
    # )

    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution * dist_unit
    y = math.sin(theta1) * planContribution * dist_unit
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)) * dist_unit

    return [x, y, z]


def computeDKDetailed(
    theta1,
    theta2,
    theta3,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    use_rads=USE_RADS_INPUT,
    use_mm=USE_MM_OUTPUT,
):
    theta1_verif = theta1
    theta2_verif = theta2
    theta3_verif = theta3
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit

    # print(
    #     "corrected angles={}, {}, {}".format(
    #         theta1 * (1.0 / angle_unit),
    #         theta2 * (1.0 / angle_unit),
    #         theta3 * (1.0 / angle_unit),
    #     )
    # )

    planContribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * planContribution
    y = math.sin(theta1) * planContribution
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3))

    p0 = [0, 0, 0]
    p1 = [l1 * math.cos(theta1) * dist_unit, l1 * math.sin(theta1) * dist_unit, 0]
    p2 = [
        (l1 + l2 * math.cos(theta2)) * math.cos(theta1) * dist_unit,
        (l1 + l2 * math.cos(theta2)) * math.sin(theta1) * dist_unit,
        -l2 * math.sin(theta2) * dist_unit,
    ]
    p3 = [x * dist_unit, y * dist_unit, z * dist_unit]
    p3_verif = computeDK(
        theta1_verif, theta2_verif, theta3_verif, l1, l2, l3, use_rads, use_mm
    )
    if (p3[0] != p3_verif[0]) or (p3[1] != p3_verif[1]) or (p3[2] != p3_verif[2]):
        print(
            "ERROR: the DK function is broken!!! p3 = {}, p3_verif = {}".format(
                p3, p3_verif
            )
        )

    return [p0, p1, p2, p3]


# Computes the inverse kinematics of a leg in the leg's frame
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def computeIK(
    x,
    y,
    z,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    verbose=False,
    use_rads=USE_RADS_OUTPUT,
    sign=-1,
    use_mm=USE_MM_INPUT,
):
    dist_unit = 1
    if use_mm:
        dist_unit = 0.001
    x = x * dist_unit
    y = y * dist_unit
    z = z * dist_unit

    # theta1 is simply the angle of the leg in the X/Y plane. We have the first angle we wanted.
    if y == 0 and x == 0:
        # Taking care of this singularity (leg right on top of the first rotational axis)
        theta1 = 0
    else:
        theta1 = math.atan2(y, x)

    # Distance between the second motor and the projection of the end of the leg on the X/Y plane
    xp = math.sqrt(x * x + y * y) - l1
    # if xp < 0:
    #     print("Destination point too close")
    #     xp = 0

    # Distance between the second motor arm and the end of the leg
    d = math.sqrt(math.pow(xp, 2) + math.pow(z, 2))
    # if d > l2 + l3:
    #     print("Destination point too far away")
    #     d = l2 + l3

    # Knowing l2, l3 and d, theta1 and theta2 can be computed using the Al Kashi law
    # There are 2 solutions for most of the points, forcing a convention here
    theta2 = alKashi(l2, d, l3, sign=sign) - Z_DIRECTION * math.atan2(z, xp)
    theta3 = math.pi + alKashi(l2, l3, d, sign=sign)

    if use_rads:

        result = [
            angleRestrict(THETA1_MOTOR_SIGN * theta1, use_rads=use_rads),
            angleRestrict(
                THETA2_MOTOR_SIGN * (theta2 + theta2Correction), use_rads=use_rads
            ),
            angleRestrict(
                THETA3_MOTOR_SIGN * (theta3 + theta3Correction), use_rads=use_rads
            ),
        ]

    else:
        result = [
            angleRestrict(THETA1_MOTOR_SIGN * math.degrees(theta1), use_rads=use_rads),
            angleRestrict(
                THETA2_MOTOR_SIGN * (math.degrees(theta2) + theta2Correction),
                use_rads=use_rads,
            ),
            angleRestrict(
                THETA3_MOTOR_SIGN * (math.degrees(theta3) + theta3Correction),
                use_rads=use_rads,
            ),
        ]
    if verbose:
        print(
            "Asked IK for x={}, y={}, z={}\n, --> theta1={}, theta2={}, theta3={}".format(
                x, y, z, result[0], result[1], result[2],
            )
        )

    return result


# Computes the inverse kinematics of a leg in a frame colinear to the robot's frame (x points in front of the robot, y points to its left, z towards the sky)
# but whose (0,0) point is leg dependent, ie will match the leg's initial position.
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def computeIKOriented(x, y, z, legID, extra_theta=0, verbose=False, use_rads=True, offsets=[0, 0 ,0]):
    new_pos = rotaton_2D(x, y, z, -LEG_ANGLES[legID] + extra_theta)

    alphas = computeDK(0, 0, 0, use_rads=True)
    
    new_pos[0] += alphas[0] + offsets[0]
    new_pos[1] += alphas[1] + offsets[1]
    new_pos[2] += alphas[2] + offsets[2]

    result = computeIK(new_pos[0], new_pos[1], new_pos[2], verbose=verbose, use_rads=use_rads)

    return result


# Computes the inverse kinematics of a leg in a frame colinear to the leg's frame (x points in front of the leg, y points to its left, z towards the sky)
# but whose (0,0) point matches the leg's initial position.
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def computeIKNotOriented(x, y, z, legID, verbose=False, use_rads=True):

    alphas = computeDK(0, 0, 0, use_rads=True)
    
    x += alphas[0]
    y += alphas[1]

    result = computeIK(x, y, z, verbose=verbose, use_rads=use_rads)

    return result


def rotaton_2D(x, y, z, theta):
    # Applying a rotation around the Z axis

    new_x = x * math.cos(theta) - y * math.sin(theta)
    new_y = x * math.sin(theta) + y * math.cos(theta)

    return [new_x, new_y, z]


def angleRestrict(angle, use_rads=False):
    if use_rads:
        return modulopi(angle)
    else:
        return modulo180(angle)


# Takes an angle that's between 0 and 360 and returns an angle that is between -180 and 180
def modulo180(angle):
    if -180 < angle < 180:
        return angle

    angle = angle % 360
    if angle > 180:
        return -360 + angle

    return angle


def modulopi(angle):
    if -math.pi < angle < math.pi:
        return angle

    angle = angle % (math.pi * 2)
    if angle > math.pi:
        return -math.pi * 2 + angle

    return angle

def segment(x1, y1, z1, x2, y2, z2, t, duration, mode='segment', oriented=False, legID=0):
    
    # new_t = ((cos(((2*pi)/duration) * t) + 1) * duration) / 2

    # new_t : triangle wave form from 0 to duration
    new_t = math.fmod(t, duration)

    if new_t > duration/2 and mode == 'segment':
        new_t = duration/2 - (new_t - duration/2)
        new_t *= 2

    x = (x2 - x1) * new_t/duration + x1
    y = (y2 - y1) * new_t/duration + y1
    z = (z2 - z1) * new_t/duration + z1

    if oriented:
        # new_positions = rotaton_2D(x, y, z, math.pi/2-LEG_ANGLES[legID])
        # x = new_positions[0]
        # y = new_positions[1]
        return computeIKOriented(-x, y, z, legID)

    results = computeIK(x, y, z)

    return results


def trianglePoints(x, z, h, w):
    """
    Takes the geometric parameters of the triangle and returns the position of the 3 points of the triagles. Format : [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]
    """
    None


def triangle(x, z, h, w, t, duration=3, oriented=False, legID=0, offset=0):
    """
    Takes the geometric parameters of the triangle and the current time, gives the joint angles to draw the triangle with the tip of th leg. Format : [theta1, theta2, theta3]
    """
    seg_duration = duration / 3
    h += z
    new_t = math.fmod(t, duration)

    y_min = -w/2 + offset
    y_max = w/2 + offset
    
    while 0 <= new_t <= seg_duration:
        return segment(x, y_min, z, x, y_max, z, t, seg_duration, 'triangle', oriented, legID)
    while seg_duration < new_t <= 2*seg_duration:
        return segment(x, y_max, z, x, 0+offset, h, t, seg_duration, 'triangle', oriented, legID)
    while 2*seg_duration < new_t <= duration:
        return segment(x, 0+offset, h, x, y_min, z, t, seg_duration, 'triangle', oriented, legID)


### Only used for walk
def triangle_walk(x, z, h, w, t, duration=3, oriented=False, legID=0, offset=0, sens=1):
    seg_duration = duration / 3
    h += z
    new_t = math.fmod(t, duration)

    y = x

    x_min = (-w/2 + offset) * sens
    x_max = (w/2 + offset) * sens
    
    while 0 <= new_t <= seg_duration:
        return segment(x_min, y, z, x_max, y, z, t, seg_duration, 'triangle', oriented, legID)
    while seg_duration < new_t <= 2*seg_duration:
        return segment(x_max, y, z, 0+offset, y, h, t, seg_duration, 'triangle', oriented, legID)
    while 2*seg_duration < new_t <= duration:
        return segment(0+offset, y, h, x_min, y, z, t, seg_duration, 'triangle', oriented, legID)


def circlePoints(x, z, r, N=16):
    """
    Takes the geometric parameters of the cercle and returns N points approximating the circle. Format : [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], etc]
    """
    None


"""Autre façon de faire le cercle (à présenter)
            # y_circle = r * math.cos(2 * math.pi * (1 / duration) * sim.t)
            # z_circle = r * math.sin(2 * math.pi * (1 / duration) * sim.t)
            # alphas = kinematics.computeIK(x, y_circle, z_circle + z)
"""


def circle(x, z, r, t, duration):
    """
    Takes the geometric parameters of the circle and the current time, gives the joint angles to draw the circle with the tip of th leg. Format : [theta1, theta2, theta3]
    """
    y = r * math.cos(2*math.pi*(t/duration))
    z += r * math.sin(2*math.pi*(t/duration))

    results = computeIK(x, y, z)

    return results

def spider(x, y, z):

    f = 0.5

    offsets = [0, 0, 0.05]

    x = -x * math.sin(2*math.pi*f*time.time())
    y = -y * math.cos(2*math.pi*f*time.time())
    z = -z * math.cos(2*math.pi*f*time.time())

    leg_1 = computeIKOriented(x, y, z, 0, offsets=offsets)
    leg_2 = computeIKOriented(x, y, z, 1, offsets=offsets)
    leg_3 = computeIKOriented(x, y, z, 2, offsets=offsets)
    leg_4 = computeIKOriented(x, y, z, 3, offsets=offsets)
    leg_5 = computeIKOriented(x, y, z, 4, offsets=offsets)
    leg_6 = computeIKOriented(x, y, z, 5, offsets=offsets)

    return leg_1, leg_2, leg_3, leg_4, leg_5, leg_6


def testPos(x, y, z):

    leg_1 = computeIK(x, y, z)
    leg_2 = computeIK(x, -y, z)
    leg_3 = computeIK(x, -y, z)
    leg_4 = computeIK(x, -y, z)
    leg_5 = computeIK(x, y, z)
    leg_6 = computeIK(x, y, z)

    return [leg_1, leg_2, leg_3, leg_4, leg_5, leg_6]


def testLine(x, y, z):

    leg_1 = computeIKOriented(x, y, z, 0, extra_theta=-LEG_ANGLES[0])
    leg_2 = computeIKOriented(x, -y, z, 1, extra_theta=-LEG_ANGLES[1])
    leg_3 = computeIKOriented(x, -y, z, 2)
    leg_4 = computeIKOriented(x, -y, z, 3, extra_theta=-LEG_ANGLES[0])
    leg_5 = computeIKOriented(x, y, z, 4, extra_theta=-LEG_ANGLES[1])
    leg_6 = computeIKOriented(x, y, z, 5)

    return [leg_1, leg_2, leg_3, leg_4, leg_5, leg_6]


def walk(t, d, sens=1):

    orient = True

    decalage = -0.05

    # Old walking way, without IKOriented
    # right_group = -0.10
    # left_group = -right_group

    # leg_1 = triangle(right_group, -0.167, 0.05, 0.1, t+d/2, d, orient, 0, offset=-decalage)
    # leg_2 = triangle(left_group, -0.167, 0.05, 0.1, t, d, orient, 1, offset=-decalage)
    # leg_3 = triangle(left_group, -0.167, 0.05, 0.1, t+d/2, d, orient, 2)
    # leg_4 = triangle(left_group, -0.167, 0.05, 0.1, t, d, orient, 3, offset=decalage)
    # leg_5 = triangle(right_group, -0.167, 0.05, 0.1, t+d/2, d, orient, 4, offset=decalage)
    # leg_6 = triangle(right_group, -0.167, 0.05, 0.1, t, d, orient, 5)

    right_group = 0
    left_group = -right_group
    z = 0.05

    leg_1 = triangle_walk(right_group, z, 0.05, 0.1, t+d/2, d, orient, 0, offset=-decalage, sens=sens)
    leg_2 = triangle_walk(left_group, z, 0.05, 0.1, t, d, orient, 1, offset=-decalage, sens=sens)
    leg_3 = triangle_walk(left_group, z, 0.05, 0.1, t+d/2, d, orient, 2, sens=sens)
    leg_4 = triangle_walk(left_group, z, 0.05, 0.1, t, d, orient, 3, offset=decalage, sens=sens)
    leg_5 = triangle_walk(right_group, z, 0.05, 0.1, t+d/2, d, orient, 4, offset=decalage, sens=sens)
    leg_6 = triangle_walk(right_group, z, 0.05, 0.1, t, d, orient, 5, sens=sens)

    return leg_1, leg_2, leg_3, leg_4, leg_5, leg_6


def rotate(t, d, angle):

    orient = False

    right_group = 0.10
    left_group = right_group

    leg_1 = triangle(right_group, -0.167, 0.05, 0.1, t+d/2, d, orient, 0)
    leg_2 = triangle(left_group, -0.167, 0.05, 0.1, t, d, orient, 1)
    leg_3 = triangle(left_group, -0.167, 0.05, 0.1, t+d/2, d, orient, 2)
    leg_4 = triangle(left_group, -0.167, 0.05, 0.1, t, d, orient, 3)
    leg_5 = triangle(right_group, -0.167, 0.05, 0.1, t+d/2, d, orient, 4)
    leg_6 = triangle(right_group, -0.167, 0.05, 0.1, t, d, orient, 5)

    return leg_1, leg_2, leg_3, leg_4, leg_5, leg_6


def main():
    print(
        "0, -90, -90 --> ", computeDK(0, -90, -90, l1=constL1, l2=constL2, l3=constL3)
    )
    print("0, 0, 0 --> ", computeDK(0, 0, 0, l1=constL1, l2=constL2, l3=constL3))
    print("90, 0, 0 --> ", computeDK(90, 0, 0, l1=constL1, l2=constL2, l3=constL3))
    print(
        "180, -30.501, -67.819 --> ",
        computeDK(180, -30.501, -67.819, l1=constL1, l2=constL2, l3=constL3),
    )
    print(
        "0, -30.645, 38.501 --> ",
        computeDK(0, -30.645, 38.501, l1=constL1, l2=constL2, l3=constL3),
    )


if __name__ == "__main__":
    main()

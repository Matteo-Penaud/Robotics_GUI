#!/usr/bin/env python
import math
import sys
import os
import time
import argparse
from numpy.testing._private.parameterized import param
import pybullet as p
from onshape_to_robot.simulation import Simulation
from scipy.spatial.transform import rotation
import kinematics
from constants import * 

# from squaternion import Quaternion
from scipy.spatial.transform import Rotation

from kinematics import computeDK

class Parameters:
    def __init__(self):
        self.legs = []
        self.legs.append(["j_c1_rf", "j_thigh_rf", "j_tibia_rf"])
        self.legs.append(["j_c1_lf", "j_thigh_lf", "j_tibia_lf"])
        self.legs.append(["j_c1_lm", "j_thigh_lm", "j_tibia_lm"])
        self.legs.append(["j_c1_lr", "j_thigh_lr", "j_tibia_lr"])
        self.legs.append(["j_c1_rr", "j_thigh_rr", "j_tibia_rr"])
        self.legs.append(["j_c1_rm", "j_thigh_rm", "j_tibia_rm"])

params = Parameters()

global sim
old_distances = [[0, 0, 0], 
                 [0, 0, 0], 
                 [0, 0, 0], 
                 [0, 0, 0], 
                 [0, 0, 0], 
                 [0, 0, 0]]

old_linear_robot_pos = [(0, 0, 0), 0]

def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat

def init_simulation():
    global sim

    # m_friction
    controls = {}
    robotPath = "phantomx_description/urdf/phantomx.urdf"
    sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
    #sim.setFloorFrictions(lateral=1, spinning=0.1, rolling=0.1)
    pos = sim.getRobotPose()
    sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

    leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]
    leg_angle = -math.pi / 4

    targets = {}
    for name in sim.getJoints():
        if "c1" in name or "thigh" in name or "tibia" in name:
            targets[name] = 0

    return controls, sim, pos, leg_center_pos, leg_angle, targets


def reset_robot(sim, targets):
    global params
    sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

    for i in range(0, 6):
        for j in range(0, 3):
            targets[params.legs[i][j]] = 0

    state = sim.setJoints(targets)
    sim.tick()

def distance_leg(leg0, leg1):
    result = []
    result.insert(0, abs(leg0[0] - leg1[0]))
    result.insert(1, abs(leg0[1] - leg1[1]))
    result.insert(2, abs(leg0[2] - leg1[2]))

    return result

def calc_speeds(linear_pos, old_linear_pos):
    result = []

    elapsed_time = linear_pos[1] - old_linear_pos[1]

    speed_x = 0
    speed_y = 0
    speed_z = 0

    if elapsed_time != 0:
        speed_x = (linear_pos[0][0] - old_linear_pos[0][0]) / elapsed_time
        speed_y = (linear_pos[0][1] - old_linear_pos[0][1]) / elapsed_time
        speed_z = (linear_pos[0][2] - old_linear_pos[0][2]) / elapsed_time
    
    result.insert(0, speed_x)
    result.insert(1, speed_y)
    result.insert(2, speed_z)

    return result


def walk(sim, targets, speed=1, rotation_speed=0, direction=0, robot_height=0.05, step_height=0.01, step_lenght=0.15, debug=False, frozen=0, send_drift=False, legs_offset=0.02, walk_mode='triangle', rotate=0):
        global params
        global old_distances
        global old_linear_robot_pos

        drift = False

        alphas = kinematics.walk(sim.t, duration=speed, direction=direction, robot_height=robot_height, step_height=step_height, step_lenght=step_lenght, legs_offset=legs_offset, rotate=rotate)
            
        targets["j_c1_rf"] = alphas[0][0]
        targets["j_thigh_rf"] = alphas[0][1]
        targets["j_tibia_rf"] = alphas[0][2]

        targets["j_c1_lf"] = alphas[1][0]
        targets["j_thigh_lf"] = alphas[1][1]
        targets["j_tibia_lf"] = alphas[1][2]

        targets["j_c1_lm"] = alphas[2][0]
        targets["j_thigh_lm"] = alphas[2][1]
        targets["j_tibia_lm"] = alphas[2][2]

        targets["j_c1_lr"] = alphas[3][0]
        targets["j_thigh_lr"] = alphas[3][1]
        targets["j_tibia_lr"] = alphas[3][2]

        targets["j_c1_rr"] = alphas[4][0]
        targets["j_thigh_rr"] = alphas[4][1]
        targets["j_tibia_rr"] = alphas[4][2]

        targets["j_c1_rm"] = alphas[5][0]
        targets["j_thigh_rm"] = alphas[5][1]
        targets["j_tibia_rm"] = alphas[5][2]
    

        robot_position = sim.getRobotPose()
        current_linear_robot_pos = [robot_position[0], sim.t]
        lin_speeds = calc_speeds(current_linear_robot_pos, old_linear_robot_pos)
        old_linear_robot_pos = [robot_position[0], sim.t]

        sim.lookAt(robot_position[0])
        state = sim.setJoints(targets)

        if debug:
            for leg_id in range(0, 6):
                pos = kinematics.computeDK(state[params.legs[leg_id][0]][0], state[params.legs[leg_id][1]][0], state[params.legs[leg_id][2]][0])
                pos = kinematics.rotaton_2D(pos[0], pos[1], pos[2], LEG_ANGLES[leg_id]+robot_position[1][2])

                new_centers = kinematics.rotaton_2D(LEG_CENTER_POS[leg_id][0], LEG_CENTER_POS[leg_id][1], LEG_CENTER_POS[leg_id][2], robot_position[1][2])

                pos[0] += new_centers[0] + robot_position[0][0]
                pos[1] += new_centers[1] + robot_position[0][1]
                pos[2] += new_centers[2] + robot_position[0][2]

                sim.addDebugPosition(pos, duration=1.5)

        if frozen == 1:
            sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        if send_drift:
            positions = []

            for leg_id in range(0, 6):
                pos = kinematics.computeDK(state[params.legs[leg_id][0]][0], state[params.legs[leg_id][1]][0], state[params.legs[leg_id][2]][0])
                pos = kinematics.rotaton_2D(pos[0], pos[1], pos[2], LEG_ANGLES[leg_id]+robot_position[1][2])

                new_centers = kinematics.rotaton_2D(LEG_CENTER_POS[leg_id][0], LEG_CENTER_POS[leg_id][1], LEG_CENTER_POS[leg_id][2], robot_position[1][2])

                pos[0] += new_centers[0] + robot_position[0][0]
                pos[1] += new_centers[1] + robot_position[0][1]
                pos[2] += new_centers[2] + robot_position[0][2]

                positions.append(pos)

            distances = []

            distances.insert(0, distance_leg(positions[0], positions[2]))
            distances.insert(1, distance_leg(positions[2], positions[4]))
            distances.insert(2, distance_leg(positions[4], positions[0]))
            distances.insert(3, distance_leg(positions[1], positions[3]))
            distances.insert(4, distance_leg(positions[3], positions[5]))
            distances.insert(5, distance_leg(positions[5], positions[1]))

            for i in range(0, 6):
                for j in range(0, 3):
                    if abs(distances[i][j] - old_distances[i][j]) > 0.001:
                        drift = True
                    old_distances[i][j] = distances[i][j]

        sim.tick()

        return drift, lin_speeds

def rotate(sim, targets, speed=1, direction=0, robot_height=0, step_height=0, debug=False, frozen=0):

        alphas = kinematics.rotate(sim.t, duration=speed, direction=direction, robot_height=robot_height, step_height=step_height)

        targets["j_c1_rf"] = alphas[0][0]
        targets["j_thigh_rf"] = alphas[0][1]
        targets["j_tibia_rf"] = alphas[0][2]

        targets["j_c1_lf"] = alphas[1][0]
        targets["j_thigh_lf"] = alphas[1][1]
        targets["j_tibia_lf"] = alphas[1][2]

        targets["j_c1_lm"] = alphas[2][0]
        targets["j_thigh_lm"] = alphas[2][1]
        targets["j_tibia_lm"] = alphas[2][2]

        targets["j_c1_lr"] = alphas[3][0]
        targets["j_thigh_lr"] = alphas[3][1]
        targets["j_tibia_lr"] = alphas[3][2]

        targets["j_c1_rr"] = alphas[4][0]
        targets["j_thigh_rr"] = alphas[4][1]
        targets["j_tibia_rr"] = alphas[4][2]

        targets["j_c1_rm"] = alphas[5][0]
        targets["j_thigh_rm"] = alphas[5][1]
        targets["j_tibia_rm"] = alphas[5][2]

        robot_position = sim.getRobotPose()

        state = sim.setJoints(targets)
        
        if debug:
            for leg_id in range(0, 6):
                pos = kinematics.computeDK(state[params.legs[leg_id][0]][0], state[params.legs[leg_id][1]][0], state[params.legs[leg_id][2]][0])
                pos = kinematics.rotaton_2D(pos[0], pos[1], pos[2], LEG_ANGLES[leg_id]+robot_position[1][2])

                new_centers = kinematics.rotaton_2D(LEG_CENTER_POS[leg_id][0], LEG_CENTER_POS[leg_id][1], LEG_CENTER_POS[leg_id][2], robot_position[1][2])

                pos[0] += new_centers[0] + robot_position[0][0]
                pos[1] += new_centers[1] + robot_position[0][1]
                pos[2] += new_centers[2] + robot_position[0][2]

                sim.addDebugPosition(pos, duration=1.5)

        if frozen == 1:
            sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        sim.tick()

def move_legs(sim, targets, legID=0, leg_target_x=0, leg_target_y=0, leg_target_z=0, debug=False, frozen=0):
        global params

        alphas = kinematics.computeIKOriented(leg_target_x, leg_target_y, leg_target_z, legID)

        targets[params.legs[legID][0]] = alphas[0]
        targets[params.legs[legID][1]] = alphas[1]
        targets[params.legs[legID][2]] = alphas[2]

        robot_position = sim.getRobotPose()
        state = sim.setJoints(targets)
        
        if debug:
            pos = kinematics.computeDK(state[params.legs[legID][0]][0], state[params.legs[legID][1]][0], state[params.legs[legID][2]][0])
            pos = kinematics.rotaton_2D(pos[0], pos[1], pos[2], LEG_ANGLES[legID]+robot_position[1][2])

            new_centers = kinematics.rotaton_2D(LEG_CENTER_POS[legID][0], LEG_CENTER_POS[legID][1], LEG_CENTER_POS[legID][2], robot_position[1][2])

            pos[0] += new_centers[0] + robot_position[0][0]
            pos[1] += new_centers[1] + robot_position[0][1]
            pos[2] += new_centers[2] + robot_position[0][2]

            sim.addDebugPosition(pos, duration=1.5)

        sim.lookAt(robot_position[0])

        if frozen == 1:
            sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
        sim.tick()



def move_body(sim, targets, body_target_x=0, body_target_y=0, body_target_z=0, debug=False, frozen=0):

        alphas = kinematics.spider(-body_target_x, -body_target_y, -body_target_z)

        targets["j_c1_rf"] = alphas[0][0]
        targets["j_thigh_rf"] = alphas[0][1]
        targets["j_tibia_rf"] = alphas[0][2]

        targets["j_c1_lf"] = alphas[1][0]
        targets["j_thigh_lf"] = alphas[1][1]
        targets["j_tibia_lf"] = alphas[1][2]

        targets["j_c1_lm"] = alphas[2][0]
        targets["j_thigh_lm"] = alphas[2][1]
        targets["j_tibia_lm"] = alphas[2][2]

        targets["j_c1_lr"] = alphas[3][0]
        targets["j_thigh_lr"] = alphas[3][1]
        targets["j_tibia_lr"] = alphas[3][2]

        targets["j_c1_rr"] = alphas[4][0]
        targets["j_thigh_rr"] = alphas[4][1]
        targets["j_tibia_rr"] = alphas[4][2]

        targets["j_c1_rm"] = alphas[5][0]
        targets["j_thigh_rm"] = alphas[5][1]
        targets["j_tibia_rm"] = alphas[5][2]
        
        robot_position = sim.getRobotPose()

        sim.lookAt(robot_position[0])
        state = sim.setJoints(targets)

        if debug:
            for leg_id in range(0, 6):
                pos = kinematics.computeDK(state[params.legs[leg_id][0]][0], state[params.legs[leg_id][1]][0], state[params.legs[leg_id][2]][0])
                pos = kinematics.rotaton_2D(pos[0], pos[1], pos[2], LEG_ANGLES[leg_id]+robot_position[1][2])

                new_centers = kinematics.rotaton_2D(LEG_CENTER_POS[leg_id][0], LEG_CENTER_POS[leg_id][1], LEG_CENTER_POS[leg_id][2], robot_position[1][2])

                pos[0] += new_centers[0] + robot_position[0][0]
                pos[1] += new_centers[1] + robot_position[0][1]
                pos[2] += new_centers[2] + robot_position[0][2]

                sim.addDebugPosition(pos, duration=1.5)

        if frozen == 1:
            sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        sim.tick()



def egg_wave(sim, targets, legID=0, debug=False, frozen=0):
    global params

    new_t = math.fmod(sim.t, 2)

    x = 0.1
    y = abs(0.1 * math.sin(math.pi*new_t))
    z = 0.2

    alphas = kinematics.computeIKOriented(x, y, z, legID)

    targets[params.legs[legID][0]] = alphas[0]
    targets[params.legs[legID][1]] = alphas[1]
    targets[params.legs[legID][2]] = alphas[2]

    robot_position = sim.getRobotPose()
    state = sim.setJoints(targets)
    
    if debug:
        if 0 < math.fmod(sim.t, 0.05) < 0.02:
            pos = kinematics.computeDK(state[params.legs[legID][0]][0], state[params.legs[legID][1]][0], state[params.legs[legID][2]][0])
            pos = kinematics.rotaton_2D(pos[0], pos[1], pos[2], LEG_ANGLES[legID]+robot_position[1][2])

            new_centers = kinematics.rotaton_2D(LEG_CENTER_POS[legID][0], LEG_CENTER_POS[legID][1], LEG_CENTER_POS[legID][2], robot_position[1][2])

            pos[0] += new_centers[0] + robot_position[0][0]
            pos[1] += new_centers[1] + robot_position[0][1]
            pos[2] += new_centers[2] + robot_position[0][2]

            sim.addDebugPosition(pos, duration=1.5)

    sim.lookAt(robot_position[0])

    if frozen == 1:
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
    sim.tick()


def egg_shake(sim, targets, frozen=0):
    global params

    new_t = math.fmod(sim.t, 2)

    x = 0
    y = 0
    z = 0.02 * math.sin(math.pi*new_t*4)

    for legID in range(0, 6):
        alphas = kinematics.computeIKOriented(x, y, z, legID)

        targets[params.legs[legID][0]] = alphas[0]
        targets[params.legs[legID][1]] = alphas[1]
        targets[params.legs[legID][2]] = alphas[2]

    robot_position = sim.getRobotPose()
    state = sim.setJoints(targets)
    
    sim.lookAt(robot_position[0])

    if frozen == 1:
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
    sim.tick()


def egg_twerk(sim, targets, frozen=0):
    global params

    new_t = math.fmod(sim.t, 2)

    x = 0
    y = 0
    z = 0

    for legID in range(0, 6):
        if legID == 0 or legID == 1:
            z = 0.05
            
        elif legID == 2 or legID == 5:
            z = 0.05

        elif legID == 3 or legID == 4:
            z = 0.05-abs(0.06 * math.sin(math.pi*new_t*4))

        alphas = kinematics.computeIKOriented(x, y, z, legID)

        targets[params.legs[legID][0]] = alphas[0]
        targets[params.legs[legID][1]] = alphas[1]
        targets[params.legs[legID][2]] = alphas[2]

    robot_position = sim.getRobotPose()
    state = sim.setJoints(targets)
    
    sim.lookAt(robot_position[0])

    if frozen == 1:
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
    sim.tick()


def egg_madison(sim, targets, frozen=0):
    global params

    beat = 60/121
    tempo = math.fmod(sim.t, 60/121 * 8)

    if 0 <= tempo <= 3.5*beat:
        walk(sim, targets, speed=0.5, direction=math.pi/2)
    elif 3.5*beat <= tempo <= 4*beat:
        move_legs(sim, targets, legID=1, leg_target_x=0.1, leg_target_y=0.05, leg_target_z=0.2)
    elif 4*beat <= tempo <= 7.5*beat:
        walk(sim, targets, speed=0.5, direction=-math.pi/2)
    elif 7.5*beat <= tempo <= 8*beat:
        move_legs(sim, targets, legID=0, leg_target_x=0.1, leg_target_y=-0.05, leg_target_z=0.2)


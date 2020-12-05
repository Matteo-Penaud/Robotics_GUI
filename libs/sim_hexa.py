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


def walk(sim, targets, speed=1, direction=0, robot_height=0, step_height=0, debug=False, frozen=0):
        global params

        alphas = kinematics.walk(sim.t, duration=speed, direction=direction, robot_height=robot_height, step_height=step_height)

        #alphas_rot = kinematics.rotation(sim.t, duration=speed, direction=direction, robot_height=robot_height, step_height=step_height)

        targets["j_c1_rf"] = alphas[0][0] #+ alphas_rot[0][0]
        targets["j_thigh_rf"] = alphas[0][1] #+ alphas_rot[0][1]
        targets["j_tibia_rf"] = alphas[0][2] #+ alphas_rot[0][2]

        targets["j_c1_lf"] = alphas[1][0] #+ alphas_rot[1][0]
        targets["j_thigh_lf"] = alphas[1][1] #+ alphas_rot[1][1]
        targets["j_tibia_lf"] = alphas[1][2] #+ alphas_rot[1][2]

        targets["j_c1_lm"] = alphas[2][0] #+ alphas_rot[2][0]
        targets["j_thigh_lm"] = alphas[2][1] #+ alphas_rot[2][1]
        targets["j_tibia_lm"] = alphas[2][2] #+ alphas_rot[2][2]

        targets["j_c1_lr"] = alphas[3][0] #+ alphas_rot[3][0]
        targets["j_thigh_lr"] = alphas[3][1] #+ alphas_rot[3][1]
        targets["j_tibia_lr"] = alphas[3][2] #+ alphas_rot[3][2]

        targets["j_c1_rr"] = alphas[4][0] #+ alphas_rot[4][0]
        targets["j_thigh_rr"] = alphas[4][1] #+ alphas_rot[4][1]
        targets["j_tibia_rr"] = alphas[4][2] #+ alphas_rot[4][2]

        targets["j_c1_rm"] = alphas[5][0] #+ alphas_rot[5][0]
        targets["j_thigh_rm"] = alphas[5][1] #+ alphas_rot[5][1]
        targets["j_tibia_rm"] = alphas[5][2] #+ alphas_rot[5][2]
        
        robot_position = sim.getRobotPose()

        sim.lookAt(robot_position[0])
        state = sim.setJoints(targets)

        if debug:
            if 0 < math.fmod(sim.t, 0.05) < 0.02:
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

def rotate(sim, targets, debug=False, frozen=0):

        alphas = kinematics.rotate(sim.t, duration=3)

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
            if 0 < math.fmod(sim.t, 0.05) < 0.02:
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

def static(sim, targets, legID=0, leg_target_x=0, leg_target_y=0, leg_target_z=0, debug=False, frozen=0):
        global params

        alphas = kinematics.computeIKOriented(leg_target_x, leg_target_y, leg_target_z, legID)

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

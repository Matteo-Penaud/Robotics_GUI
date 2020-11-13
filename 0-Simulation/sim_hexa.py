#!/usr/bin/env python
import math
import sys
import os
import time
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation
from scipy.spatial.transform import rotation
import kinematics
from constants import * 

# from squaternion import Quaternion
from scipy.spatial.transform import Rotation

from kinematics import computeDK


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
    # m_friction
    controls = {}
    robotPath = "phantomx_description/urdf/phantomx.urdf"
    sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
    #sim.setFloorFrictions(lateral=1, spinning=0.1, rolling=0.1)
    pos = sim.getRobotPose()
    sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

    leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]
    leg_angle = -math.pi / 4

    sim_start_time = time.time()

    return controls, sim, pos, leg_center_pos, leg_angle, sim_start_time

while True:
    targets = {}
    for name in sim.getJoints():
        if "c1" in name or "thigh" in name or "tibia" in name:
            targets[name] = 0
    if args.mode == "frozen-direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        points = kinematics.computeDKDetailed(
            targets["j_c1_rf"],
            targets["j_thigh_rf"],
            targets["j_tibia_rf"],
            use_rads=True,
        )
        i = -1
        T = []
        for pt in points:
            # Drawing each step of the DK calculation
            i += 1
            T.append(kinematics.rotaton_2D(pt[0], pt[1], pt[2], leg_angle))
            T[-1][0] += leg_center_pos[0]
            T[-1][1] += leg_center_pos[1]
            T[-1][2] += leg_center_pos[2]
            # print("Drawing cross {} at {}".format(i, T))
            p.resetBasePositionAndOrientation(
                crosses[i], T[-1], to_pybullet_quaternion(0, 0, leg_angle)
            )

        # Temp
        sim.setRobotPose([0, 0, 0.5], to_pybullet_quaternion(0, 0, 0))
        # sim.setRobotPose(
        #     leg_center_pos, to_pybullet_quaternion(0, 0, 0),
        # )
        state = sim.setJoints(targets)
    elif args.mode == "direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        state = sim.setJoints(targets)
    elif args.mode == "inverse":
        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])
        alphas = kinematics.computeIK(x, y, z, 0, verbose=False)

        dk0 = kinematics.computeDK(0, 0, 0, use_rads=True)
        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = alphas[1]
        targets["j_tibia_rf"] = alphas[2]

        state = sim.setJoints(targets)
        # Temp
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        T = kinematics.rotaton_2D(x, y, z, leg_angle)
        T[0] += leg_center_pos[0]
        T[1] += leg_center_pos[1]
        T[2] += leg_center_pos[2]
        # print("Drawing cross {} at {}".format(i, T))
        p.resetBasePositionAndOrientation(
            cross, T, to_pybullet_quaternion(0, 0, leg_angle)
        )

    elif args.mode == "spider":

        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])

        alphas = kinematics.spider(x, y, z)

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

        for leg in range(6):
            robpos = sim.getRobotPose()
            position = kinematics.computeDK(alphas[leg][0], alphas[leg][1], alphas[leg][2])
            position = kinematics.rotaton_2D(position[0], position[1], position[2], LEG_ANGLES[leg])
            position[0] += LEG_CENTER_POS[leg][0] + robpos[0][0]
            position[1] += LEG_CENTER_POS[leg][1] + robpos[0][1]
            position[2] += LEG_CENTER_POS[leg][2] + robpos[0][2]
            positions[leg] = position
    
        for position in positions:
            sim.addDebugPosition(position, duration=1.5)

        state = sim.setJoints(targets)
        #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

    elif args.mode == "walk":

        #entryTime = time.time() - sim_start_time

        speed = p.readUserDebugParameter(controls["Speed"])

        alphas = kinematics.walk(sim.t, speed)

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

        # for leg in range(6):
        #     robpos = sim.getRobotPose()
        #     position = kinematics.computeDK(alphas[leg][0], alphas[leg][1], alphas[leg][2])
        #     position = kinematics.rotaton_2D(position[0], position[1], position[2], LEG_ANGLES[leg])
        #     position[0] += LEG_CENTER_POS[leg][0] + robpos[0][0]
        #     position[1] += LEG_CENTER_POS[leg][1] + robpos[0][1]
        #     position[2] += LEG_CENTER_POS[leg][2] + robpos[0][2]
        #     positions[leg] = position
    
        # for position in positions:
        #     sim.addDebugPosition(position, duration=1.5)
        
        state = sim.setJoints(targets)

        # sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        # leavingTime = time.time() - sim_start_time

        # frequency = 1/(leavingTime - entryTime)

        # print("Entry at : {:.5}s, leaving at : {:.5}s".format(entryTime, leavingTime))
        # print("Frequency of {:.5} Hz\n".format(frequency))


    elif args.mode == "rotate":

        alphas = kinematics.rotate(sim.t, 3, 90)

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
        
        #sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

        state = sim.setJoints(targets)

    elif args.mode == "test-all":

        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])

        alphas = kinematics.testPos(x, y, z)

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

        for leg in range(0, 6, 1):
            pos = kinematics.computeDK(alphas[leg][0], alphas[leg][1], alphas[leg][2])
            pos = kinematics.rotaton_2D(pos[0], pos[1], pos[2], LEG_ANGLES[leg])
            pos[0] += LEG_CENTER_POS[leg][0]
            pos[1] += LEG_CENTER_POS[leg][1]
            pos[2] += LEG_CENTER_POS[leg][2] + 0.5
            sim.addDebugPosition(pos, duration=3)

        state = sim.setJoints(targets)

        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
        

    elif args.mode == "test-line":

        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])

        alphas = kinematics.testLine(x, y, z)

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

        state = sim.setJoints(targets)

        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

    sim.tick()


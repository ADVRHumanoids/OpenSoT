#!/usr/bin/env python

import os
import yarp
import numpy as np
from __future__ import print_function
import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

__author__ = 'Alessio Rocchi'

def yarpPose2KDLPose(poseBottle):
    l = poseBottle.get(1).asDouble()
    v = kdl.Vector(l.get(1), l.get(2), l.get(3))
    r = kdl.Rotation.EulerZYX(l.get(4), l.get(5), l.get(6))

    return kdl.Frame(r, v)

def KDLPose2yarpPose(kdlPose, bottle):
    bottle.clear()

    tmp0 = bottle.addList()
    tmp0.addString("frame")
    tmp0.addString("world")
    p = kdlPose.p
    r = kdlPose.M.getEulerZYX()
    tmp = bottle_right.addList()
    tmp.addString("data")
    tmp.addDouble(p[0])
    tmp.addDouble(p[1])
    tmp.addDouble(p[2])
    tmp.addDouble(r[0])
    tmp.addDouble(r[1])
    tmp.addDouble(r[2])

    return bottle

def yarpListToTuple(listBottle):
    tuple = []
    for i in range(listBottle.size()):
        tuple += [listBottle.get(i)]
    return tuple

if __name__ == '__main__':

    print "This Test creates Trajectories for left and right arm end-effectors"

    yarp.Network.init()

    coman_urdf = os.environ.get('YARP_WORKSPACE') + '/IITComanRosPkg/coman_urdf/urdf/coman.urdf'
    print('Trying to open', coman_urdf)
    f = file(coman_urdf, 'r')
    robot = Robot.from_xml_string(f.read())
    f.close()

    tree = kdl_tree_from_urdf_model(robot)
    print(tree.getNrOfSegments())

    base_link = robot.get_root()
    end_link = 'r_wrist'
    chain = tree.getChain(base_link, end_link)
    print('Right arm & waist have:', chain.getNrOfJoints(), 'joints')

    kdl_kin = KDLKinematics(robot, base_link, end_link)


    q_reader_right_arm = yarp.BufferedPortBottle()
    q_reader_right_arm.open("/reader/right_arm/state:i")
    yarp.Network.connect("/reader/right_arm/state:i", "/coman/right_arm/state:o")

    q_reader_torso = yarp.BufferedPortBottle()
    q_reader_torso.open("/reader/torso/state:i")
    yarp.Network.connect("/reader/torso/state:i", "/coman/torso/state:o")

    world_pose_reader = yarp.BufferedPortBottle()
    world_pose_reader.open("/reader/world_to_base_link_pose:i")
    yarp.Network.connect("/reader/world_to_base_link_pose:i", "/sot_VelKinCon/world_to_base_link_pose:o")


    world_pose = world_pose_reader.read()

    right_arm = yarp.BufferedPortBottle()
    right_arm.open("/desired/right_arm/position/ref:o")
    yarp.Network.connect("/desired/right_arm/position/ref:o", "/sot_VelKinCon/right_arm/set_ref:i")

    left_arm_x_f = 0.119692
    left_arm_y_f = 0.189703
    left_arm_z_f = 0.819815
    left_arm_R_f = 0.0
    left_arm_P_f = -45.0
    left_arm_Y_f = 0.0

    r = 0.08

    q_torso = yarpListToTuple(q_reader_torso.read())
    q_right_arm = yarpListToTuple(q_reader_right_arm.read())

    q = np.array(q_torso+q_right_arm)

    world_pose = yarpPose2KDLPose(world_pose_reader.read())
    pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)
    pose_w = world_pose*pose
    r_wrist_pose_0 = pose_w

    print('right arm + torso q at t0 are:', q)

    print('r_wrist pose at t0 wrt world is:', r_wrist_pose_0)

    t_start = yarp.Time.now()
    while(1):
        t = yarp.Time.now() - t_start
        l = 0.5 * (-np.cos( 0.4*t%(2 * np.pi) ) + 1)

        pose_offset = kdl.Frame()
        pose_offset.p = kdl.Vector(0,0,l)

        r_wrist_pose_t = r_wrist_pose_0*pose_offset

        bottle_right = right_arm.prepare()

        KDLPose2yarpPose(r_wrist_pose_t,bottle_right)

        right_arm.write()
        yarp.Time.delay(0.01)

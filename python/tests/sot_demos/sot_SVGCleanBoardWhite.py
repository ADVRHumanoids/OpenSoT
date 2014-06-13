#!/usr/bin/env python
from __future__ import print_function
import os
import yarp
import numpy as np
import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from svg.path import parse_path, Path, Line, QuadraticBezier
from xml.dom import minidom

__author__ = 'Alessio Rocchi'

def numPyToFrame(mat):
    v = kdl.Vector(mat[0,3],mat[1,3],mat[2,3])
    r = kdl.Rotation()
    frame = kdl.Frame(r, v)
    for i in range(3):
        for j in range(3):
            frame.M[i,j] = mat[i,j]

    return frame

def yarpPose2KDLPose(poseBottle):
    l = poseBottle.get(1).asList()
    v = kdl.Vector(l.get(1).asDouble(), l.get(2).asDouble(), l.get(3).asDouble())
    #r = kdl.Rotation.RPY(l.get(4).asDouble(), l.get(5).asDouble(), l.get(6).asDouble())
    r = kdl.Rotation()
    return kdl.Frame(r, v)

def KDLPose2yarpPose(kdlPose, bottle):
    bottle.clear()

    tmp0 = bottle.addList()
    tmp0.addString("frame")
    tmp0.addString("world")
    p = kdlPose.p
    r = kdlPose.M.GetQuaternion()
    tmp = bottle_right.addList()
    tmp.addString("data")
    tmp.addDouble(p[0])
    tmp.addDouble(p[1])
    tmp.addDouble(p[2])
    tmp.addDouble(r[0])
    tmp.addDouble(r[1])
    tmp.addDouble(r[2])
    tmp.addDouble(r[3])

    return bottle

def yarpListToTuple(listBottle):
    tuple = []
    for i in range(listBottle.size()):
        tuple += [listBottle.get(i).asDouble()]
    return tuple

if __name__ == '__main__':

    print("This Test creates Trajectories for left and right arm end-effectors")

    yarp.Network.init()

    coman_urdf = os.environ.get('YARP_WORKSPACE') + '/IITComanRosPkg/coman_urdf/urdf/coman.urdf'
    print('Trying to open', coman_urdf)
    f = file(coman_urdf, 'r')
    robot = Robot.from_xml_string(f.read())
    f.close()

    rf = yarp.ResourceFinder()
    rf.setDefaultContext('sot_VelKinCon')
    svg_file = rf.findPath('whiteboard.svg')
    svg = minidom.parse(svg_file)
    svg_el = svg.getElementsByTagName('svg')
    svg_width = svg_el.item(0).getAttribute('width')
    svg_height = svg_el.item(0).getAttribute('height')
    paths = svg.getElementsByTagName('path')
    if paths.length > 0:
        path_data = paths.item(0).getAttribute('d')
        path = parse_path(path_data)
    else:
        print('Error getting path, exiting')
        exit()


    tree = kdl_tree_from_urdf_model(robot)
    print(tree.getNrOfSegments())

    base_link = robot.get_root()
    #base_link = 'base_link'
    end_link = 'r_wrist'
    chain = tree.getChain(base_link, end_link)
    print('Right arm & waist have:', chain.getNrOfJoints(), 'joints')

    kdl_kin = KDLKinematics(robot, base_link, end_link)


    q_reader_right_arm = yarp.BufferedPortBottle()
    q_reader_right_arm.open("/reader/right_arm/state:i")
    yarp.Network.connect("/coman/right_arm/state:o", "/reader/right_arm/state:i")

    q_reader_torso = yarp.BufferedPortBottle()
    q_reader_torso.open("/reader/torso/state:i")
    yarp.Network.connect("/coman/torso/state:o", "/reader/torso/state:i")

    world_pose_reader = yarp.BufferedPortBottle()
    world_pose_reader.open("/reader/world_to_base_link_pose:i")
    yarp.Network.connect("/sot_VelKinCon/world_to_base_link_pose:o", "/reader/world_to_base_link_pose:i")

    right_arm = yarp.BufferedPortBottle()
    right_arm.open("/desired/right_arm/position/ref:o")
    yarp.Network.connect("/desired/right_arm/position/ref:o", "/sot_VelKinCon/right_arm/set_ref:i")

    q_torso = yarpListToTuple(q_reader_torso.read())
    q_right_arm = yarpListToTuple(q_reader_right_arm.read())

    q = np.array(q_torso+q_right_arm) * np.pi/180

    world_pose = yarpPose2KDLPose(world_pose_reader.read())
    poseNumpy = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 numpy.mat)
    pose = numPyToFrame(poseNumpy)

    print("world pose is:\n", world_pose)
    print("r_wrist wrt base_link is:\n", pose)

    w_pose = world_pose*pose
    r_wrist_pose_0 = w_pose

    print('right arm + torso q at t0 are:\n', q)

    print('r_wrist pose at t0 wrt world is:\n', r_wrist_pose_0)

    print('accessing path..')
    y_path_0 = path.point(0).imag
    z_path_0 = path.point(0).real

    t_start = yarp.Time.now()
    print('starting trajectory generation')
    while(1):
        t = yarp.Time.now() - t_start

        # scaling time so that in 10 seconds we go from 0 to 1
        t_scaled = 1 - abs(np.cos( 0.1/2 * t%(2 * np.pi) ))

        # we will go through the path, scaled to be 10cm wide
        y_path = (path.point(t_scaled).imag - y_path_0)/float(svg_width)*.1
        z_path = (path.point(t_scaled).real - z_path_0)/float(svg_height)*.1

        print("SVG: going to:", y_path, z_path)

        pose_offset = kdl.Frame()
        pose_offset.p = kdl.Vector(0, y_path, z_path)

        r_wrist_pose_t = pose_offset*r_wrist_pose_0

        bottle_right = right_arm.prepare()

        KDLPose2yarpPose(r_wrist_pose_t, bottle_right)

        print("Desired Pose = ", r_wrist_pose_t)

        right_arm.write()

        yarp.Time.delay(0.01)
        #yarp.Time.delay(1)

#!/usr/bin/python

__author__ = ('Alessio Rocchi', 'Enrico Mingo')

import yarp
import numpy as np
import PyKDL as kdl
import pYTask
import rospy
import tf
import subprocess

class TrajectoryGenerator(object):
    def __init__(self, duration, current_pose, desired_pose):
        self.duration = float(duration)
        self.initial_pose = current_pose
        self.desired_pose = desired_pose

        print "Creating trajectory generator from ", current_pose, " to ", desired_pose
        print "Trajectory will take ", duration, "s"

    def Duration(self):
        return self.duration

    def Pos(self, t):

        _lambda = np.min((t / self.duration, 1.0))
        d = kdl.Frame()
        d.p = (1-_lambda)*self.initial_pose.p + _lambda*self.desired_pose.p
        R = self.initial_pose.M.Inverse()*self.desired_pose.M
        #v = R.GetRot()
        (alpha,v) = R.GetRotAngle()
        dR = kdl.Rotation.Rot(v,_lambda*alpha)
        d.M = self.initial_pose.M * dR

        return d

def generate_traj_msg(source_frame, target_frame, trajGen, t, msg_port):

    desired_pose = trajGen.Pos(t)
    
    bottle_desired_pose_msg = msg_port.prepare()
    bottle_desired_pose_msg.clear()
    pYTask.pose_msg(desired_pose, source_frame, target_frame, bottle_desired_pose_msg)
    return bottle_desired_pose_msg


print "OpenSoT python Example using pYTask. Please run the executable example_python before running this script"

if __name__ == '__main__':
    rospy.init_node('wall_task')

    #p = subprocess.Popen(["./example_python"])

    yarp.Network.init()
    while not yarp.Time.isValid():
        continue

    port_r_wrist = yarp.BufferedPortBottle()
    port_r_wrist.open("/bigman/example_python/cartesian::r_wrist/set_ref:o")
    yarp.Network.connect("/bigman/example_python/cartesian::r_wrist/set_ref:o",
                         "/bigman/example_python/cartesian::r_wrist/set_ref:i")

    port_waist = yarp.BufferedPortBottle()
    port_waist.open("/bigman/example_python/cartesian::waist/set_ref:o")
    yarp.Network.connect("/bigman/example_python/cartesian::waist/set_ref:o",
                         "/bigman/example_python/cartesian::waist/set_ref:i")

    port_l_wrist = yarp.BufferedPortBottle()
    port_l_wrist.open("/bigman/example_python/cartesian::l_wrist/set_ref:o")
    yarp.Network.connect("/bigman/example_python/cartesian::l_wrist/set_ref:o",
                         "/bigman/example_python/cartesian::l_wrist/set_ref:i")

    listener = tf.TransformListener()

    timeoutDuration = rospy.Duration(secs=10)
    source_frame='world'
    target_frame_l='l_wrist'
    target_frame_r='r_wrist'
    target_frame_waist='Waist'

    try:
        listener.waitForTransform(source_frame, target_frame_l, time=rospy.Time(0), timeout=timeoutDuration)
        listener.waitForTransform(source_frame, target_frame_r, time=rospy.Time(0), timeout=timeoutDuration)
        listener.waitForTransform(source_frame, target_frame_waist, time=rospy.Time(0), timeout=timeoutDuration)
    except tf.Exception as e:
        print("Error on waitForTransform:%s"%e.message)

    success = False
    while not success:
        try:
            (world_pos_l_wrist, world_rot_l_wrist) = listener.lookupTransform(source_frame, target_frame_l, rospy.Time(0))
            (world_pos_r_wrist, world_rot_r_wrist) = listener.lookupTransform(source_frame, target_frame_r, rospy.Time(0))
            (world_pos_waist, world_rot_waist) = listener.lookupTransform(source_frame, target_frame_waist, rospy.Time(0))
            success = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Error on lookupTransform")
            continue

    world_l_wrist_pose = kdl.Frame(kdl.Rotation.Quaternion(world_rot_l_wrist[0],
                                                           world_rot_l_wrist[1],
                                                           world_rot_l_wrist[2],
                                                           world_rot_l_wrist[3]),
                                   kdl.Vector(world_pos_l_wrist[0],
                                              world_pos_l_wrist[1],
                                              world_pos_l_wrist[2]))
    world_r_wrist_pose = kdl.Frame(kdl.Rotation.Quaternion(world_rot_r_wrist[0],
                                                           world_rot_r_wrist[1],
                                                           world_rot_r_wrist[2],
                                                           world_rot_r_wrist[3]),
                                   kdl.Vector(world_pos_r_wrist[0],
                                              world_pos_r_wrist[1],
                                              world_pos_r_wrist[2]))
    world_waist_pose = kdl.Frame(kdl.Rotation.Quaternion(world_rot_waist[0],
                                                         world_rot_waist[1],
                                                         world_rot_waist[2],
                                                         world_rot_waist[3]),
                                 kdl.Vector(world_pos_waist[0],
                                            world_pos_waist[1],
                                            world_pos_waist[2]))


    world_l_wrist_desired = world_l_wrist_pose
    world_l_wrist_desired.p[0] -= 0.25
    world_l_wrist_desired.p[1] += 0.15
    world_l_wrist_desired.p[2] -= 0.75
    world_l_wrist_desired.M.DoRotY(np.pi/2)
    world_r_wrist_desired = world_r_wrist_pose
    world_r_wrist_desired.p[1] += 0.4
    world_r_wrist_desired.p[1] -= 0.15
    world_r_wrist_desired.p[2] -= 0.3
    world_waist_desired = world_waist_pose
    world_waist_desired.p[2] -= 0.4
    #world_r_wrist_desired.M.DoRotY(np.pi/2)

    duration = 100000.0  # apparently. yarp.os.Time is not working properly...
    t_init = yarp.Time.now()
    t_elapsed = 0.0
    l_wrist_traj_gen = TrajectoryGenerator(duration, world_l_wrist_pose, world_l_wrist_desired)
    r_wrist_traj_gen = TrajectoryGenerator(duration, world_r_wrist_pose, world_r_wrist_desired)
    waist_traj_gen = TrajectoryGenerator(duration, world_waist_pose, world_waist_desired)
    while t_elapsed < duration:
        generate_traj_msg(source_frame, target_frame_l,
                          l_wrist_traj_gen, t_elapsed,
                          port_l_wrist)
        port_l_wrist.write()

        generate_traj_msg(source_frame, target_frame_r,
                          r_wrist_traj_gen, t_elapsed,
                          port_r_wrist)
        port_r_wrist.write()

        generate_traj_msg(source_frame, target_frame_waist,
                          waist_traj_gen, t_elapsed,
                          port_waist)
        port_waist.write()

        t_elapsed = yarp.Time.now() - t_init
        yarp.Time.delay(0.01)

    #p.terminate()
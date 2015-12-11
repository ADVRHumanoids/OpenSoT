#!/usr/bin/python

__author__ = ('Alessio Rocchi', 'Enrico Mingo')

import yarp
import numpy as np
import PyKDL as kdl
import pYTask

import subprocess

class LinearTrajectory(object):
    def __init__(self, duration, current_pose, desired_pose, loop = False):
        self.duration = float(duration)
        self.initial_pose = current_pose
        self.desired_pose = desired_pose
        self.loop = loop

        print "Creating trajectory generator for line segment going from \n", current_pose, " to \n", desired_pose
        print "Trajectory will take ", duration, "s"

    def Duration(self):
        return self.duration

    def Pos(self, t):
        if self.loop is True:
            _lambda = t / self.duration
            while _lambda > 2.0:
                _lambda /= 2.0
            if _lambda > 1.0:
                _lambda = 2.0 - _lambda
        else:
            _lambda = np.min((t / self.duration, 1.0))
        d = kdl.Frame()
        d.p = (1-_lambda)*self.initial_pose.p + _lambda*self.desired_pose.p
        R = self.initial_pose.M.Inverse()*self.desired_pose.M
        #v = R.GetRot()
        (alpha,v) = R.GetRotAngle()
        dR = kdl.Rotation.Rot(v,_lambda*alpha)
        d.M = self.initial_pose.M * dR

        return d


print "OpenSoT python Example using pYTask. Please run the executable example_python before running this script"

if __name__ == '__main__':

    #p = subprocess.Popen(["./example_python"])

    yarp.Network.init()
    while not yarp.Time.isValid():
        continue

    l_wrist = pYTask.CartesianTask('example_python',
                                   'bigman',
                                   'cartesian::l_wrist',
                                   'world', 'l_wrist')

    l_wrist_act = l_wrist.getActualPose()
    l_wrist_des = kdl.Frame(l_wrist_act)
    l_wrist_des.p[0] += 0.15
    l_wrist_des.p[1] += 0.05
    l_wrist_des.p[2] -= 0.50
    l_wrist_des.M.DoRotZ(-np.pi/3)

    duration = 10.0  # apparently. yarp.os.Time is not working properly...
    t_init = yarp.Time.now()
    t_elapsed = 0.0
    l_wrist_traj_gen = LinearTrajectory(duration, l_wrist_act, l_wrist_des, True)
    print "actual pose:\n", l_wrist_act
    print "desired pose:\n", l_wrist_des
    while t_elapsed < 2*duration:
        ref = l_wrist_traj_gen.Pos(t_elapsed)
        print "commaded pose @t=", t_elapsed, "=\n",ref

        l_wrist.setReference(l_wrist_traj_gen.Pos(t_elapsed))
        t_elapsed = yarp.Time.now() - t_init

        yarp.Time.delay(0.035)

    #p.terminate()
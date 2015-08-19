__author__ = 'Alessio Rocchi'
import PyKDL as kdl
import numpy as np


class CircularTrajectory(object):
    def __init__(self, duration, current_pose, center_pose, loop=False):
        """
        Creates a circular trajectory; orientation is kept from the current_pose, and the position
        navigates a circular path with center on center_pose and radius given by the distance between the two frames
        :param duration: the trajectory duration
        :param current_pose: a kdl.Frame representing the actual position on the circle
        :param center_pose: a kdl.Frame representing the desired center of the circle.
                            The circle will be drawn about the z-axis of the center Frame
        :return:
        """

        self.duration = float(duration)
        self.w_T_ee = current_pose
        self.radius = (center_pose.p - current_pose.p).Norm()
        self.w_T_c = center_pose

        assert(self.w_T_c.p != self.w_T_ee.p)
        assert( np.abs(kdl.dot((center_pose.p - current_pose.p), self.w_T_c.M.UnitZ())) < 1.0)

        print "Creating trajectory generator for circle with center:"
        print center_pose
        print "Trajectory will take ", duration, "s"

    def Duration(self):
        return self.duration

    def Pos(self, t):
        _lambda = np.min((t / self.duration, 1.0))
        dR = kdl.Frame()
        dR.M.DoRotZ(2.0 * np.pi * _lambda)
        c_T_ee = self.w_T_c.Inverse()*self.w_T_ee
        pose = self.w_T_c * dR * c_T_ee
        pose.M = self.w_T_ee.M

        return pose

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

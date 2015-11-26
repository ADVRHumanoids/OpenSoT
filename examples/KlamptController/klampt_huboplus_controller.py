from controller import *
from klampt import *
import time
import os
import numpy as np
from urdf_parser_py.urdf import URDF

import pYTask
import ExampleKlamptController as OpenSoT
from collections import Counter

DEBUG_MODE = False

class KlamptJointInfo(object):
    def __init__(self, robot_klampt, robot_urdf_path):
        self.robot = robot_klampt
        self.urdf = URDF.from_xml_file(robot_urdf_path)

    def printJntMap(self):
        for i in xrange(self.robot.numDrivers()):
            link = self.robot.getDriver(i).getName()
            parent_joint, parent_link = self.urdf.parent_map[link]
            print i, ": ", link, "->", parent_joint

    def jntMapToKlampt(self, jntMap):
        q = np.zeros(self.robot.numDrivers())
        for i in xrange(self.robot.numDrivers()):
            link = self.robot.getDriver(i).getName()
            parent_joint, parent_link = self.urdf.parent_map[link]
            q[i] = jntMap[parent_joint]
        return q

    def klamptToJntMap(self, joints):
        assert(joints is not None)
        assert(len(joints) == self.robot.numDrivers())
        jntMap = dict()
        for i in xrange(self.robot.numDrivers()):
            link = self.robot.getDriver(i).getName()
            parent_joint, parent_link = self.urdf.parent_map[link]
            #from IPython.core.debugger import Tracer
            #Tracer()()
            jntMap[parent_joint] = joints[i]
        return OpenSoT.JntMap(jntMap)


class HuboPlusController(BaseController):
    """A controller for the HuboPlus"""
    def __init__(self, robot):
        self.robot = robot
        self.startTime = None
        self.realStartTime = time.time()
        self.sot_controller = None
        self.posture = None
        # TODO change absolute path with relative, take into account argv[0]
        robURDFPath = os.path.normpath(
            os.path.join(__file__,'../../../data/robots/huboplus/huboplus.urdf'))
        self.jntMapper = KlamptJointInfo(self.robot, robURDFPath)

    def output(self,**inputs):
        api = ControllerAPI(inputs)
        t = api.time()

        if self.startTime is None:
            self.startTime = t

        if self.posture is None:
            q = api.sensedConfiguration()
            if q is not None:
                if len(q) == 0:
                    return None
                print "Number of Drives:", self.robot.numDrivers()
                print "Number of Joint:", len(q)
                print "Link to Joint map:"
                self.jntMapper.printJntMap()

                self.posture = self.jntMapper.klamptToJntMap(q)
                self.sot_controller = OpenSoT.ExampleKlamptController(self.posture)
            else:
                print "Number of Drives:", self.robot.numDrivers()
                #print "Number of Joint:", len(joints)
                print "Link to Joint map:"
                self.jntMapper.printJntMap()

                self.sot_controller = OpenSoT.ExampleKlamptController()
                self.posture = self.sot_controller.getPosture()
                print "Error: sensedConfiguration is empty"
                print "Using initial posture from controller:", self.posture.asdict()

        t = t - self.startTime

        # Main loop - not CLIK
        if self.posture is not None:
            dq = self.sot_controller.computeControl(self.posture)
            if DEBUG_MODE:
                print "Output:", dq.asdict()

            accumulator = Counter(self.posture.asdict())
            accumulator.update(Counter(dq.asdict()))
            self.posture = OpenSoT.JntMap(accumulator)
            if DEBUG_MODE:
                print "Integrated Posture:", self.posture.asdict()

            q_cmd = self.jntMapper.jntMapToKlampt(self.posture)
            if DEBUG_MODE:
                print "Final Command:", q_cmd
        else:
            q_cmd = np.zeros(self.robot.numDrivers())

        return api.makePositionCommand(q_cmd)

def make(robot):
    return HuboPlusController(robot)

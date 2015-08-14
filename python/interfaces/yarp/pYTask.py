import yarp
import numpy as np
import PyKDL as kdl

def pose_msg(kdlFrame, base_frame, distal_frame, bottle):
    bottle.clear()

    p = kdlFrame.p
    r = kdlFrame.M.GetQuaternion()

    bottle.addDouble(p[0])
    bottle.addDouble(p[1])
    bottle.addDouble(p[2])
    bottle.addDouble(r[0])
    bottle.addDouble(r[1])
    bottle.addDouble(r[2])
    bottle.addDouble(r[3])
    bottle.addString(base_frame)
    bottle.addString(distal_frame)

    return bottle

def twist_msg(kdlTwist, base_frame, distal_frame, bottle):
    bottle.clear()

    v = kdlTwist.vel
    w = kdlTwist.rot

    bottle.addDouble(v[0])
    bottle.addDouble(v[1])
    bottle.addDouble(v[2])
    bottle.addDouble(w[0])
    bottle.addDouble(w[1])
    bottle.addDouble(w[2])
    bottle.addString(base_frame)
    bottle.addString(distal_frame)

    return bottle

def position_joint_msg(joint_name_list, joint_value_list, bottle):
    if(len(joint_name_list) == len(joint_value_list)):
        bottle.clear()

        for i in range(len(joint_name_list)):
            bottle.addString(joint_name_list[i])
            bottle.addDouble(joint_value_list[i])

        return bottle

def trj_msg(kdlFrame, kdlTwist, base_frame, distal_frame, bottle):
    bottle.clear()

    pose_msg(kdlFrame, base_frame, distal_frame, bottle)
    twist_msg(kdlTwist, base_frame, distal_frame, bottle)

    return bottle

class YTask(object):
    def __init__(self, module_name, robot_name, task_name):
        port_prefix = '/'+robot_name+'/'+module_name+'/'+task_name+'/'
        self.set_ref = yarp.BufferedPortBottle()
        self.set_ref.open(port_prefix+'set_ref:o')
        yarp.Network.connect(port_prefix+'set_ref:o',
                             port_prefix+'set_ref:i')
        self.rpc = yarp.RpcClient()
        self.rpc.open(port_prefix+'rpc_client')
        self.rpc.addOutput(port_prefix+'rpc')
        #yarp.Network.connect(port_prefix+'rpc_client',
        #                     port_prefix+'rpc')

    def help(self):
        reply = yarp.Bottle()
        reply.clear()

        request = yarp.Bottle()
        request.clear()

        request.addString('help')
        self.rpc.write(request, reply)

        return reply

    def setW(self, W):
        return

    def getW(self):
        return

    def setLambda(self, l):
        return

    def getLambda(self):
        return


class CartesianTask(YTask):
    def __init__(self, module_name, robot_name, task_name):
        super(CartesianTask, self).__init__(module_name, robot_name, task_name)

    def setReference(self, ref_pose):
        pass

    def getActualPose(self):
        reply = yarp.Bottle()
        reply.clear()

        request = yarp.Bottle()
        request.clear()

        request.addString('get actual_pose')
        self.rpc.write(request, reply)
        return reply

    def setOrientationGain(self, orientation_gain):
        pass

    def getOrientationGain(self, orientation_gain):
        pass

class CoMTask(YTask):
    def __init__(self, module_name, robot_name, task_name):
        super(CoMTask, self).__init__(module_name, robot_name, task_name)

    def setReference(self, ref_position):
        pass

    def getActualPosition(self):
        reply = yarp.Bottle()
        reply.clear()

        request = yarp.Bottle()
        request.clear()

        request.addString('get')
        request.addString('actual_position')
        self.rpc.write(request, reply)
        return reply

class PosturalTask(YTask):
    def __init__(self, module_name, robot_name, task_name):
        super(PosturalTask, self).__init__(module_name, robot_name, task_name)

    def setReference(self, ref_posture):
        pass

    def getActualPosture(self):
        reply = yarp.Bottle()
        reply.clear()

        request = yarp.Bottle()
        request.clear()

        request.addString('get')
        request.addString('actual_position')
        self.rpc.write(request, reply)
        return reply

__author__ = ('Alessio Rocchi', 'Enrico Mingo')

if __name__ == '__main__':

    print('pYTask')

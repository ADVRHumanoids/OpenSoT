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
        self.set_ref.addOutput(port_prefix+'set_ref:i')
        #yarp.Network.connect(port_prefix+'set_ref:o',
        #                     port_prefix+'set_ref:i')
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
        pass

    def getW(self):
        pass

    def setLambda(self, l):
        reply = yarp.Bottle()
        reply.clear()

        request = yarp.Bottle()
        request.clear()

        request.addString('set lambda')
        request.addDouble(l)
        self.rpc.write(request, reply)

    def getLambda(self):
        reply = yarp.Bottle()
        reply.clear()

        request = yarp.Bottle()
        request.clear()

        request.addString('get lambda')
        self.rpc.write(request, reply)
        return reply.get(0).asDouble()


class CartesianTask(YTask):
    def __init__(self, module_name, robot_name, task_name, base_link, distal_link):
        super(CartesianTask, self).__init__(module_name, robot_name, task_name)
        self.base = base_link
        self.distal = distal_link

    def setReference(self, ref_pose):
        """
        Sets a reference pose for the robot ee of this cartesian task
        :param ref_pose: a kdl.Frame with the desired pose of the EE
        :return: True on success
        """

        cmd = self.set_ref.prepare()
        pose_msg(ref_pose, self.base, self.distal, cmd)

        return self.set_ref.write()

    def getActualPose(self):
        """
        Returns the actual pose of the EE
        :return: a kdl.Frame with the actual pose of the EE
        """
        reply = yarp.Bottle()
        reply.clear()

        request = yarp.Bottle()
        request.clear()

        request.addString('get actual_pose')
        self.rpc.write(request, reply)
        if reply.size() != 16:
            raise Exception('Unexpected reply size')

        frame = kdl.Frame()

        for i in xrange(3):
            frame.p[i] = reply.get(i*4 + 3).asDouble()

        for r in xrange(3):
            for c in xrange(3):
                frame.M[r, c] = reply.get(r*4 + c).asDouble()

        return frame

    def setOrientationGain(self, orientation_gain):
        pass

    def getOrientationGain(self, orientation_gain):
        pass

class CoMTask(YTask):
    def __init__(self, module_name, robot_name, task_name):
        super(CoMTask, self).__init__(module_name, robot_name, task_name)
        self.base = 'world'
        self.distal = 'CoM'

    def setReference(self, ref_position):
        cmd = self.set_ref.prepare()

        ref_pose = kdl.Frame()
        ref_pose.p = ref_position
        pose_msg(ref_pose, self.base, self.distal, cmd)

        return self.set_ref.write()

    def getActualPosition(self):
        reply = yarp.Bottle()
        reply.clear()

        request = yarp.Bottle()
        request.clear()

        request.addString('get actual_position')
        self.rpc.write(request, reply)

        position = kdl.Vector()

        if reply.size() != 3:
            raise Exception('Unexpected reply size')

        for i in xrange(3):
            position[i] = reply.get(i).asDouble()

        return position

class PosturalTask(YTask):
    def __init__(self, module_name, robot_name, task_name):
        super(PosturalTask, self).__init__(module_name, robot_name, task_name)

    def setReference(self, ref_posture):
        """

        :param ref_posture: a dict {'jnt_name0':q_cmd0,'jnt_name1':q_cmd1}
        :return:
        """

        cmd = self.set_ref.prepare()
        position_joint_msg(ref_posture.keys(), ref_posture.values(), cmd)

        return self.set_ref.write(cmd)


    def getActualPosture(self):
        """
        Returns the posture of the robot as a numpy array
        :return: a dict {'jnt_name0':q0,'jnt_name1':q_1,...,'jnt_namen_1':q_n_1}
        """
        reply = yarp.Bottle()
        reply.clear()

        request = yarp.Bottle()
        request.clear()

        request.addString('get actual_posture')
        self.rpc.write(request, reply)

        posture = dict()
        for i in xrange(reply.size()/2):
            posture[reply.get(2*i).asString()] = reply.get(2*i+1).asDouble()
        return posture

__author__ = ('Alessio Rocchi', 'Enrico Mingo')

if __name__ == '__main__':

    print('pYTask')

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

def trj_msg(kdlFrame, kdlTwist, base_frame, distal_frame, bottle):
    bottle.clear()

    pose_msg(kdlFrame, base_frame, distal_frame, bottle)
    twist_msg(kdlTwist, base_frame, distal_frame, bottle)

    return bottle

__author__ = ('Alessio Rocchi', 'Enrico Mingo')

if __name__ == '__main__':

    print('pYTask')

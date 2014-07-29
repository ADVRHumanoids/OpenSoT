#!/usr/bin/python

import yarp
import numpy as np
import rospy
import tf
import PyKDL as kdl
from pydrc import pydrc

print "testCoM"

if __name__ == '__main__':
    rospy.init_node('testCoM')

    yarp.Network.init()

    CoM = yarp.BufferedPortBottle()
    CoM.open("/desired/CoM/position/ref:o")
    yarp.Network.connect("/desired/CoM/position/ref:o", "/sot_VelKinCon/com/set_ref:i")

    listener = tf.TransformListener()
    timeoutDuration = rospy.Duration(secs=10)
    listener.waitForTransform(target_frame='/l_sole', source_frame='/CoM', time=rospy.Time(0), timeout=timeoutDuration)

    success = False
    while not success:
        try:
            (com_pos_i, com_rot_i) = listener.lookupTransform('/l_sole', '/CoM', rospy.Time(0))
            success = True
            print "CoM position ", com_pos_i
            print "CoM orientation ", com_rot_i
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    if success:
        print("SUCCESS on lookupTransform")

        com_pos_0 = kdl.Frame()
        com_pos_0.p = kdl.Vector(com_pos_i[0], com_pos_i[1], com_pos_i[2])
        com_pos_0.M = kdl.Rotation.Identity()

        com_pos_f = kdl.Frame()
        com_pos_f.p = kdl.Vector(com_pos_i[0], com_pos_i[1] + 0.08, com_pos_i[2] - 0.07)
        com_pos_f.M = kdl.Rotation.Identity()

        t_start = yarp.Time.now()
        while not rospy.is_shutdown():
            t = yarp.Time.now() - t_start
            l = 0.5 * (-np.cos( 0.6*t%(2 * np.pi) ) + 1)

            com_x = com_pos_0.p[0]# + l*(com_pos_f.p[0] - com_pos_0.p[0])
            com_y = com_pos_0.p[1]# + l*(com_pos_f.p[1] - com_pos_0.p[1])
            com_z = com_pos_0.p[2] + l*(com_pos_f.p[2] - com_pos_0.p[2])

            com_pos = kdl.Frame()
            com_pos.p = kdl.Vector(com_x, com_y, com_z)
            com_pos.M = kdl.Rotation.Identity()

            bottle_CoM = CoM.prepare()
            bottle_CoM.clear()

            pydrc.KDLPose2yarpPose(com_pos, bottle_CoM, "l_sole")
            CoM.write()

            yarp.Time.delay(0.01)
    else:
        print("ERROR on lookupTransform")
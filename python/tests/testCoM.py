#!/usr/bin/python

import yarp
import numpy as np
import rospy
import tf


print "testCoM"

if __name__ == '__main__':
    rospy.init_node('testCoM')

    yarp.Network.init()

    CoM = yarp.BufferedPortBottle()
    CoM.open("/desired/CoM/position/ref:o")
    yarp.Network.connect("/desired/CoM/position/ref:o", "/sot_VelKinCon/com/set_ref:i")

    listener = tf.TransformListener()

    success = False
    i = 0
    while not success:
        try:
            (com_pos_0, com_rot_0) = listener.lookupTransform('/l_sole', '/CoM', rospy.Time(0))
            success = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            i = i + 1
            if i == 2000:
                break
            continue

    if success:
        print("SUCCESS on lookupTransform")

        com_x_f = com_pos_0[0]
        com_y_f = com_pos_0[1] + 0.11
        com_z_f = com_pos_0[2] - 0.07

        t_start = yarp.Time.now()
        while not rospy.is_shutdown():
            t = yarp.Time.now() - t_start
            l = 0.5 * (-np.cos( 0.6*t%(2 * np.pi) ) + 1)

            com_x = com_pos_0[0]# + l*(com_x_f - com_pos_0[0])
            com_y = com_pos_0[1]# + l*(com_y_f - com_pos_0[1])
            com_z = com_pos_0[2] + l*(com_z_f - com_pos_0[2])

            bottle_CoM = CoM.prepare()
            bottle_CoM.clear()

            bottle_tmp0 = bottle_CoM.addList()
            bottle_tmp0.addString("frame")
            bottle_tmp0.addString("l_sole")

            bottle_tmp = bottle_CoM.addList()
            bottle_tmp.addString("data")
            bottle_tmp.addDouble(com_x)
            bottle_tmp.addDouble(com_y)
            bottle_tmp.addDouble(com_z)
            bottle_tmp.addDouble(0.0)
            bottle_tmp.addDouble(0.0)
            bottle_tmp.addDouble(0.0)

            CoM.write()

            yarp.Time.delay(0.01)
    else:
        print("ERROR on lookupTransform")
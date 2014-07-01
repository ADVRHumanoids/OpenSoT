#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import yarp

print("Test TF")

if __name__ == '__main__':
    rospy.init_node('testTf')

    yarp.Network.init()

    CoM = yarp.BufferedPortBottle()
    CoM.open("/desired/CoM/position/ref:o")
    yarp.Network.connect("/desired/CoM/position/ref:o", "/sot_VelKinCon/com/set_ref:i")

    listener = tf.TransformListener()

    success = False
    while not success:
        try:
            (l_sole_pos_CoM, l_sole_rot_CoM) = listener.lookupTransform('/l_sole', '/CoM', rospy.Time(0))
            success = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("ERROR on lookupTransform")
            continue

    print("CoM initial position is:")
    print l_sole_pos_CoM

    com_x_d = l_sole_pos_CoM[0]
    com_y_d= l_sole_pos_CoM[1]
    com_z_d = l_sole_pos_CoM[2] - 0.05

    bottle_CoM = CoM.prepare()
    bottle_CoM.clear()

    bottle_tmp0 = bottle_CoM.addList()
    bottle_tmp0.addString("frame")
    bottle_tmp0.addString("l_sole")

    bottle_tmp = bottle_CoM.addList()
    bottle_tmp.addString("data")
    bottle_tmp.addDouble(com_x_d)
    bottle_tmp.addDouble(com_y_d)
    bottle_tmp.addDouble(com_z_d)
    bottle_tmp.addDouble(0.0)
    bottle_tmp.addDouble(0.0)
    bottle_tmp.addDouble(0.0)

    CoM.write()

    yarp.Time.delay(0.01)





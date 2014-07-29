#!/usr/bin/python

import yarp
import numpy as np
import os
import rospy
import tf

print "COM + Swing Foot Test"

if __name__ == '__main__':
    rospy.init_node('swingFootTest')

    yarp.Network.init()

    yarp.Time.isValid() # bug! We need to check network clock

    yarpClockPort = os.getenv("YARP_CLOCK")
    if not yarp.Time.isSystemClock():
        print('Using network clock!')
    else:
        print('Using system clock!')

    listener = tf.TransformListener()

    CoM = yarp.BufferedPortBottle()
    CoM.open("/desired/CoM/position/ref:o")
    yarp.Network.connect("/desired/CoM/position/ref:o", "/sot_VelKinCon/com/set_ref:i")

    swing_foot = yarp.BufferedPortBottle()
    swing_foot.open("/desired/swing_foot/pose/ref:o")
    yarp.Network.connect("/desired/swing_foot/pose/ref:o", "/sot_VelKinCon/swing_foot/set_ref:i")

    timeoutDuration = rospy.Duration(secs=10)
    listener.waitForTransform(target_frame='/l_sole', source_frame='/CoM', time=rospy.Time(0), timeout=timeoutDuration)
    listener.waitForTransform(target_frame='/l_sole', source_frame='/r_sole', time=rospy.Time(0), timeout=timeoutDuration)


    success = False
    while not success:
        try:
            (com_pos_0, com_rot_0) = listener.lookupTransform(target_frame='/l_sole', source_frame='/CoM', time=rospy.Time(0))
            success = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    success = False
    while not success:
        try:
            (r_sole_pos_0, r_sole_rot_0) = listener.lookupTransform(target_frame='/l_sole', source_frame='/r_sole', time=rospy.Time(0))
            success = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue



    #com_x_0 = 0.021886
    #com_y_0 = -0.125690
    #com_z_0 = 0.525539

    com_x_0 = com_pos_0[0]
    com_y_0 = com_pos_0[1]
    com_z_0 = com_pos_0[2]

    com_y_f = 0.0

    #swing_foot_x_0 = 0.00588938
    #swing_foot_y_0 = -0.183015
    #swing_foot_z_0 = 0.00474352
    #swing_foot_qx_0 = -0.0122438
    #swing_foot_qy_0 = -0.00106837
    #swing_foot_qz_0 = 0.00151965
    #swing_foot_qw_0 = 0.999923

    swing_foot_x_0 = r_sole_pos_0[0]
    swing_foot_y_0 = r_sole_pos_0[1]
    swing_foot_z_0 = r_sole_pos_0[2]
    swing_foot_qx_0 = r_sole_rot_0[0]
    swing_foot_qy_0 = r_sole_rot_0[1]
    swing_foot_qz_0 = r_sole_rot_0[2]
    swing_foot_qw_0 = r_sole_rot_0[3]


    swing_foot_z_f = r_sole_pos_0[2] + 0.045

    ####

    com_x = com_x_0
    com_y = com_y_f
    com_z = com_z_0

    swing_foot_x = swing_foot_x_0
    swing_foot_y = swing_foot_y_0
    swing_foot_z = swing_foot_z_f
    swing_foot_qx = swing_foot_qx_0
    swing_foot_qy = swing_foot_qy_0
    swing_foot_qz = swing_foot_qz_0
    swing_foot_qw = swing_foot_qw_0


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

    print "Move CoM"
    CoM.write()

    yarp.Time.delay(5)	# delay 15 seconds

    bottle_swing_foot = swing_foot.prepare()
    bottle_swing_foot.clear()

    bottle_tmp1 = bottle_swing_foot.addList()
    bottle_tmp1.addString("frame")
    bottle_tmp1.addString("l_sole")

    bottle_tmp2 = bottle_swing_foot.addList()
    bottle_tmp2.addString("data")
    bottle_tmp2.addDouble(swing_foot_x)
    bottle_tmp2.addDouble(swing_foot_y)
    bottle_tmp2.addDouble(swing_foot_z)
    bottle_tmp2.addDouble(swing_foot_qx)
    bottle_tmp2.addDouble(swing_foot_qy)
    bottle_tmp2.addDouble(swing_foot_qz)
    bottle_tmp2.addDouble(swing_foot_qw)

    print "Move swing foot up"
    swing_foot.write()

    yarp.Time.delay(2)

    bottle_swing_foot = swing_foot.prepare()
    bottle_swing_foot.clear()

    swing_foot_x_f = swing_foot_x_0 + 0.05

    swing_foot_x = swing_foot_x_f
    swing_foot_y = swing_foot_y_0
    swing_foot_z = swing_foot_z_f
    swing_foot_qx = swing_foot_qx_0
    swing_foot_qy = swing_foot_qy_0
    swing_foot_qz = swing_foot_qz_0
    swing_foot_qw = swing_foot_qw_0


    bottle_tmp1 = bottle_swing_foot.addList()
    bottle_tmp1.addString("frame")
    bottle_tmp1.addString("l_sole")

    bottle_tmp2 = bottle_swing_foot.addList()
    bottle_tmp2.addString("data")
    bottle_tmp2.addDouble(swing_foot_x)
    bottle_tmp2.addDouble(swing_foot_y)
    bottle_tmp2.addDouble(swing_foot_z)
    bottle_tmp2.addDouble(swing_foot_qx)
    bottle_tmp2.addDouble(swing_foot_qy)
    bottle_tmp2.addDouble(swing_foot_qz)
    bottle_tmp2.addDouble(swing_foot_qw)

    print "Move swing foot forward"
    swing_foot.write()

    yarp.Time.delay(2)

    bottle_swing_foot = swing_foot.prepare()
    bottle_swing_foot.clear()

    swing_foot_z_f = swing_foot_z_0

    swing_foot_x = swing_foot_x_f
    swing_foot_y = swing_foot_y_0
    swing_foot_z = swing_foot_z_f
    swing_foot_qx = swing_foot_qx_0
    swing_foot_qy = swing_foot_qy_0
    swing_foot_qz = swing_foot_qz_0
    swing_foot_qw = swing_foot_qw_0


    bottle_tmp1 = bottle_swing_foot.addList()
    bottle_tmp1.addString("frame")
    bottle_tmp1.addString("l_sole")

    bottle_tmp2 = bottle_swing_foot.addList()
    bottle_tmp2.addString("data")
    bottle_tmp2.addDouble(swing_foot_x)
    bottle_tmp2.addDouble(swing_foot_y)
    bottle_tmp2.addDouble(swing_foot_z)
    bottle_tmp2.addDouble(swing_foot_qx)
    bottle_tmp2.addDouble(swing_foot_qy)
    bottle_tmp2.addDouble(swing_foot_qz)
    bottle_tmp2.addDouble(swing_foot_qw)

    print "Move swing foot down"
    swing_foot.write()

    yarp.Time.delay(2)

    com_x = com_x_0 + 0.05
    com_y = com_y_0 * 1.5
    com_z = com_z_0

    swing_foot_x = swing_foot_x_0
    swing_foot_y = swing_foot_y_0
    swing_foot_z = swing_foot_z_f
    swing_foot_qx = swing_foot_qx_0
    swing_foot_qy = swing_foot_qy_0
    swing_foot_qz = swing_foot_qz_0
    swing_foot_qw = swing_foot_qw_0


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

    print "Move CoM"
    CoM.write()


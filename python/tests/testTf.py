#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import yarp

if __name__ == '__main__':
    rospy.init_node('testTf')

    yarp.Network.init()

    left_arm = yarp.BufferedPortBottle()
    left_arm.open("/desired/left_arm/position/ref:o")
    yarp.Network.connect("/desired/left_arm/position/ref:o", "/sot_VelKinCon/left_arm/set_ref:i")

    right_arm = yarp.BufferedPortBottle()
    right_arm.open("/desired/right_arm/position/ref:o")
    yarp.Network.connect("/desired/right_arm/position/ref:o", "/sot_VelKinCon/right_arm/set_ref:i")

    listener = tf.TransformListener()

    success = False
    while not success:
        try:
            (l_sole_trans_l_wrist, l_sole_rot_l_wrist) = listener.lookupTransform('/l_sole', '/l_wrist', rospy.Time(0))
            success = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    success = False
    while not success:
        try:
            (l_sole_trans_Waist, l_sole_rot_Waist) = listener.lookupTransform('/l_sole', '/Waist', rospy.Time(0))
            success = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


    world_trans_l_sole = (l_sole_trans_Waist[0], -l_sole_trans_Waist[1], 0.0)
    world_rot_l_sole = (1.0, 0.0, 0.0, 0.0)

    world_trans_l_wrist = world_trans_l_sole + l_sole_trans_l_wrist
    world_rot_l_wrist = l_sole_rot_l_wrist
    print world_trans_l_wrist
    print world_rot_l_wrist



    left_arm_x_0 = 0.119692
    left_arm_y_0 = 0.189703
    left_arm_z_0 = 0.519815
    left_arm_R_0 = 21.5
    left_arm_P_0 = -63.04
    left_arm_Y_0 = -22.06


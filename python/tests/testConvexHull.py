#!/usr/bin/env python
import rospy
import tf
import yarp
import PyKDL as kdl
from pydrc import pydrc

print("Test TF")

if __name__ == '__main__':
    rospy.init_node('testTf')

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
            (l_sole_pos_CoM, l_sole_rot_CoM) = listener.lookupTransform('/l_sole', '/CoM', rospy.Time(0))
            success = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("ERROR on lookupTransform")
            continue

    print("CoM initial position is:")
    print l_sole_pos_CoM

    com_pos_d = kdl.Frame()
    com_pos_d.p = kdl.Vector(l_sole_pos_CoM[0], l_sole_pos_CoM[1] + 1.01, l_sole_pos_CoM[2])
    com_pos_d.M = kdl.Rotation.Identity()

    bottle_CoM = CoM.prepare()
    bottle_CoM.clear()

    pydrc.KDLPose2yarpPose(com_pos_d, bottle_CoM, "l_sole")

    CoM.write()

    yarp.Time.delay(0.01)





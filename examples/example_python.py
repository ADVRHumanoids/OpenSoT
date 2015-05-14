#!/usr/bin/python

__author__ = ('Alessio Rocchi', 'Enrico Mingo')

import yarp
import numpy as np
import rospy
import tf
import PyKDL as kdl
from pydrc import pydrc
import pYTask


print "OpenSoT python Example using pYTask"

if __name__ == '__main__':
    rospy.init_node('wall_task')

    yarp.Network.init()

    port_posture = yarp.BufferedPortBottle()
    port_posture.open("/desired/posture/position/ref:o")
    yarp.Network.connect("/desired/posture/position/ref:o", "/bigman/centralized_inverse_kinematics/Postural/set_ref:i")

    port_r_wrist = yarp.BufferedPortBottle()
    port_r_wrist.open("/bigman/centralized_inverse_kinematics/cartesian::r_arm/set_ref:o")
    yarp.Network.connect("/bigman/centralized_inverse_kinematics/cartesian::r_arm/set_ref:o",
                         "/bigman/centralized_inverse_kinematics/cartesian::r_arm/set_ref:i")

    listener = tf.TransformListener()

    timeoutDuration = rospy.Duration(secs=10)
    source_frame='world'
    target_frame='r_wrist'
    listener.waitForTransform(source_frame, target_frame, time=rospy.Time(0), timeout=timeoutDuration)

    success = False
    while not success:
        try:
            (world_pos_r_wrist, world_rot_r_wrist) = listener.lookupTransform(source_frame, target_frame, rospy.Time(0))
            success = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("ERROR on lookupTransform")
            continue

    r_wrist_pos_d = kdl.Frame()
   # r_wrist_pos_d.M.DoRotY(-90.0*3.1415/180.0)
    r_wrist_pos_d.M.DoRotY(-45.0*3.1415/180.0)
   # r_wrist_pos_d.p = kdl.Vector(world_pos_r_wrist[0]+0.2, world_pos_r_wrist[1], world_pos_r_wrist[2]+0.4)
    r_wrist_pos_d.p = kdl.Vector(world_pos_r_wrist[0]+0.2, world_pos_r_wrist[1], world_pos_r_wrist[2]-1.0)

    bottle_world_pos_r_wrist = port_r_wrist.prepare()
    bottle_world_pos_r_wrist.clear()
    pYTask.pose_msg(r_wrist_pos_d, source_frame, target_frame, bottle_world_pos_r_wrist)
    port_r_wrist.write()

#    yarp.Time.delay(5)

#    r_wrist_pos_d.p = kdl.Vector(r_wrist_pos_d.p[0]+0.1, r_wrist_pos_d.p[1], r_wrist_pos_d.p[2])

#    bottle_world_pos_r_wrist = port_r_wrist.prepare()
#    bottle_world_pos_r_wrist.clear()
#    pYTask.pose_msg(r_wrist_pos_d, source_frame, target_frame, bottle_world_pos_r_wrist)
#    port_r_wrist.write()

#    yarp.Time.delay(5)

#    r_wrist_pos_d.p = kdl.Vector(r_wrist_pos_d.p[0], r_wrist_pos_d.p[1]-0.4, r_wrist_pos_d.p[2])

#    bottle_world_pos_r_wrist = port_r_wrist.prepare()
#    bottle_world_pos_r_wrist.clear()
#    pYTask.pose_msg(r_wrist_pos_d, source_frame, target_frame, bottle_world_pos_r_wrist)
#    port_r_wrist.write()

#    yarp.Time.delay(5)

#    r_wrist_pos_d.p = kdl.Vector(r_wrist_pos_d.p[0], r_wrist_pos_d.p[1], r_wrist_pos_d.p[2]-1.0)

#    bottle_world_pos_r_wrist = port_r_wrist.prepare()
#    bottle_world_pos_r_wrist.clear()
#    pYTask.pose_msg(r_wrist_pos_d, source_frame, target_frame, bottle_world_pos_r_wrist)
#    port_r_wrist.write()

#    yarp.Time.delay(5)

#    r_wrist_pos_d.p = kdl.Vector(r_wrist_pos_d.p[0], r_wrist_pos_d.p[1]+0.4, r_wrist_pos_d.p[2])

#    bottle_world_pos_r_wrist = port_r_wrist.prepare()
#    bottle_world_pos_r_wrist.clear()
#    pYTask.pose_msg(r_wrist_pos_d, source_frame, target_frame, bottle_world_pos_r_wrist)
#    port_r_wrist.write() 

 #   yarp.Time.delay(5)

 #   r_wrist_pos_d.p = kdl.Vector(r_wrist_pos_d.p[0], r_wrist_pos_d.p[1], r_wrist_pos_d.p[2]+1.0)

#    bottle_world_pos_r_wrist = port_r_wrist.prepare()
#    bottle_world_pos_r_wrist.clear()
#    pYTask.pose_msg(r_wrist_pos_d, source_frame, target_frame, bottle_world_pos_r_wrist)
#    port_r_wrist.write() 

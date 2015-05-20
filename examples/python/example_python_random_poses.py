#!/usr/bin/python

__author__ = ('Alessio Rocchi', 'Enrico Mingo')

import yarp
import numpy as np
import PyKDL as kdl
import pYTask
import rospy
import tf
import subprocess

def generate_random_pose_msg(source_frame, target_frame, current_position, current_orientation_quaternion, msg_port):
    r = np.random.rand(6)
    r -= .5
    max_variation = np.array((.5,1,1,.5,.5,.5)) #x [m],y [m],z [m],r [rad],p [rad], y [rad]
    r = r*max_variation

    desired_pose = kdl.Frame()
    desired_pose.p = kdl.Vector(current_position[0]+r[0],
                                current_position[1]+r[1],
                                current_position[2]+r[2])
    desired_pose.M = kdl.Rotation.Quaternion(current_orientation_quaternion[0],
                                             current_orientation_quaternion[1],
                                             current_orientation_quaternion[2],
                                             current_orientation_quaternion[3])
    desired_pose.M.DoRotX(r[3])
    desired_pose.M.DoRotY(r[4])
    desired_pose.M.DoRotZ(r[5])

    print desired_pose
    
    bottle_desired_pose_msg = msg_port.prepare()
    bottle_desired_pose_msg.clear()
    pYTask.pose_msg(desired_pose, source_frame, target_frame, bottle_desired_pose_msg)
    return bottle_desired_pose_msg


print "OpenSoT python Example using pYTask. Please run the executable example_python before running this script"

if __name__ == '__main__':
    rospy.init_node('wall_task')

    #p = subprocess.Popen(["./example_python"])

    yarp.Network.init()
    while not yarp.Time.isValid():
        continue

    port_r_wrist = yarp.BufferedPortBottle()
    port_r_wrist.open("/bigman/example_python/cartesian::r_wrist/set_ref:o")
    yarp.Network.connect("/bigman/example_python/cartesian::r_wrist/set_ref:o",
                         "/bigman/example_python/cartesian::r_wrist/set_ref:i")

    port_l_wrist = yarp.BufferedPortBottle()
    port_l_wrist.open("/bigman/example_python/cartesian::l_wrist/set_ref:o")
    yarp.Network.connect("/bigman/example_python/cartesian::l_wrist/set_ref:o",
                         "/bigman/example_python/cartesian::l_wrist/set_ref:i")

    listener = tf.TransformListener()

    timeoutDuration = rospy.Duration(secs=10)
    source_frame='world'
    target_frame_r='r_wrist'
    target_frame_l='l_wrist'

    try:
        listener.waitForTransform(source_frame, target_frame_r, time=rospy.Time(0), timeout=timeoutDuration)
        listener.waitForTransform(source_frame, target_frame_l, time=rospy.Time(0), timeout=timeoutDuration)
    except tf.Exception as e:
        print("Error on waitForTransform:%s"%e.message)

    success = False
    while not success:
        try:
            (world_pos_r_wrist, world_rot_r_wrist) = listener.lookupTransform(source_frame, target_frame_r, rospy.Time(0))
            (world_pos_l_wrist, world_rot_l_wrist) = listener.lookupTransform(source_frame, target_frame_l, rospy.Time(0))
            success = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Error on lookupTransform")
            continue


    for i in range(30):
        print("Sending pose %d"%i)
        generate_random_pose_msg(source_frame, target_frame_r,
                                 world_pos_r_wrist, world_rot_r_wrist,
                                 port_r_wrist)
        port_r_wrist.write()

        generate_random_pose_msg(source_frame, target_frame_l,
                                 world_pos_l_wrist, world_rot_l_wrist,
                                 port_l_wrist)
        port_l_wrist.write()

        print("Sent pose %d"%i)
        yarp.Time.delay(2.5)

    #p.terminate()
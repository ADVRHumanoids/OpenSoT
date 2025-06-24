import os
import time

import rospkg
from xbot2_interface import pyxbot2_interface as xbi
from pyopensot.tasks.velocity import Cartesian
import pyopensot as pysot
from scipy.spatial.transform import Rotation as R
import numpy as np
import rospy
from sensor_msgs.msg import JointState
import subprocess
import tf
from geometry_msgs.msg import TransformStamped

from random import seed
from random import random

# seed random number generator
seed(1)
# generate random numbers between 0-1

def generate_random_delta(min, max):
    val = np.zeros(3)
    for i in range(3):
        val[i] = random()
    return min + (val * (max - min))

# Check for franka_cartesio_condif package
try:
    package_path = rospkg.RosPack().get_path('franka_cartesio_config')
except:
    print("To run this example is needed the franka_cartesio_config package that can be download here: https://github.com/EnricoMingo/franka_cartesio_config")

# roslaunch the panda.launch with the robot description and rviz
launch_path = package_path + "/launch/panda.launch"
roslaunch = subprocess.Popen(['roslaunch', launch_path], stdout=subprocess.PIPE, shell=False)
rviz_file_path = os.getcwd() + "/panda_ik.rviz"
rviz = subprocess.Popen(['rviz',  '-d', f'{rviz_file_path}'], stdout=subprocess.PIPE, shell=False)

# Initiliaze node and wait for robot_description parameter
rospy.init_node("panda_ik", disable_signals=True)
while not rospy.has_param('/robot_description'):
    pass

# Get robot description parameter and initialize model interface (with Pinocchio)
urdf = rospy.get_param('/robot_description')
model = xbi.ModelInterface2(urdf)

# Set a homing configuration
q = [0., -0.7, 0., -2.1, 0., 1.4, 0.]
model.setJointPosition(q)
model.update()



# Create a Cartesian task at frame panda_link7, set lambda gain
c = Cartesian("Cartesian", model, "panda_link8", "world")
c.setLambda(1.)


# Get the actual reference for postural and Cartesian task
pose_ref, vel_ref = c.getReference()
dp = generate_random_delta(-0.2, 0.2)
pose_ref.translation += dp
pose_ref.linear = R.random().as_matrix()

print(f"pose_ref: {pose_ref}")

# IK loop
dt = 1./1000.
rate = rospy.Rate(1./dt)
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
msg = JointState()
msg.name = model.getJointNames()
t = 0.
gamma = 0.1
epsilon = 1e-6

br = tf.TransformBroadcaster()
w_T_b = TransformStamped()
w_T_b.header.frame_id = "world"
w_T_b.child_frame_id = "goal"

iter = 0
max_iter=10000
while not rospy.is_shutdown():
    # Update actual position in the model
    model.setJointPosition(q)
    model.update()

    c.setReference(pose_ref)
    c.update()

    J = c.getA()
    e = c.getb()
    q += gamma * np.matmul(J.transpose(), e)

    # Publish joint states
    msg.position = q
    msg.header.stamp = rospy.get_rostime()

    w_T_b.header.stamp = msg.header.stamp
    w_T_b.transform.translation.x = pose_ref.translation[0]
    w_T_b.transform.translation.y = pose_ref.translation[1]
    w_T_b.transform.translation.z = pose_ref.translation[2]
    r = R.from_matrix(pose_ref.linear)
    w_T_b.transform.rotation.x = r.as_quat()[0]
    w_T_b.transform.rotation.y = r.as_quat()[1]
    w_T_b.transform.rotation.z = r.as_quat()[2]
    w_T_b.transform.rotation.w = r.as_quat()[3]

    br.sendTransformMessage(w_T_b)

    if np.linalg.norm(e, 2) <= epsilon:
        dp = generate_random_delta(-0.2, 0.2)
        pose_ref.translation += dp
        pose_ref.linear = R.random().as_matrix()
        print(f"GOAL REACHED in {iter} iterations, error norm: {np.linalg.norm(e, 2)}")
        iter = 0
    elif iter >= max_iter:
        dp = generate_random_delta(-0.2, 0.2)
        pose_ref.translation += dp
        pose_ref.linear = R.random().as_matrix()
        print(f"MAX ITERATION REACHED, error norm: {np.linalg.norm(e, 2)}")
        iter = 0
    else:
        iter += 1

    pub.publish(msg)

    rate.sleep()

roslaunch.kill()
rviz.kill()
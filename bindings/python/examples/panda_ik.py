import os

import rospkg
from xbot2_interface import pyxbot2_interface as xbi
from pyopensot.tasks.velocity import Postural, Cartesian, Manipulability, MinimumEffort
from pyopensot.constraints.velocity import JointLimits, VelocityLimits
import pyopensot as pysot
import numpy as np
import rospy
from sensor_msgs.msg import JointState
import subprocess

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

# Get Joint Limits and Velocity Limits, define dt
qmin, qmax = model.getJointLimits()
qlims = JointLimits(model, qmax, qmin)

dqmax = model.getVelocityLimits()
dt = 1./100.
dqlims = VelocityLimits(model, dqmax, dt)

# Create postural task (it is created at the q configuration previously set
p = Postural(model)

# Create a Cartesian task at frame panda_link7, set lambda gain
c = Cartesian("Cartesian", model, "panda_link8", "world")
c.setLambda(0.1)

# Retrieve actual pose of the frame to be used as reference
ref = c.getActualPose().copy()

# Create the stack:
# 1st priority Cartesian position
# 2nd priority Cartesian orientation
# 3rd priority postural
s = ( (c%[0,1,2]) / (c%[3,4,5]) / p) << qlims
#s<<dqlims
s.update()

# Get the actual reference for postural and Cartesian task
qref, dqref = p.getReference()
print(f"qref: {qref}")
print(f"dqref: {dqref}")
pose_ref, vel_ref = c.getReference()
print(f"pose_ref: {pose_ref}")
print(f"vel_ref: {vel_ref}")

# Creates iHQP solver with stack (using qpOASES as backend)
solver = pysot.iHQP(s)
#import pyopensot_hcod
#solver = pyopensot_hcod.HCOD(s, 1e-3)


# IK loop
rate = rospy.Rate(1./dt)
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
msg = JointState()
msg.name = model.getJointNames()
t = 0.
alpha = 0.01
while not rospy.is_shutdown():
    # Update actual position in the model
    model.setJointPosition(q)
    model.update()

    # Compute new reference for Cartesian task
    pose_ref.translation[0] += alpha * np.sin(2.*3.1415 * t)
    pose_ref.translation[1] += alpha * np.cos(2.*3.1415 * t)
    t = t + alpha
    if t > 1.:
        t = 0
    c.setReference(pose_ref)

    # Update Stack
    s.update()

    # Solve
    dq = solver.solve()
    q += dq # Is fixed base so we can siply sum the result

    # Publish joint states
    msg.position = q
    msg.header.stamp = rospy.get_rostime()
    pub.publish(msg)

    rate.sleep()

roslaunch.kill()
rviz.kill()
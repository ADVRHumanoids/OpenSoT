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
    package_path = rospkg.RosPack().get_path('LittleDog')
except:
    print("To run this example is needed the anymal_c_simple_description package that can be download here: https://github.com/ANYbotics/anymal_c_simple_description")

launch_path = package_path + "/launch/LittleDog.launch"
print(launch_path)
roslaunch = subprocess.Popen(['roslaunch', launch_path], stdout=subprocess.PIPE, shell=False)
#rviz_file_path = os.getcwd() + "/panda_ik.rviz"
#rviz = subprocess.Popen(['rviz',  '-d', f'{rviz_file_path}'], stdout=subprocess.PIPE, shell=False)

# Initiliaze node and wait for robot_description parameter
rospy.init_node("LittleDog_id", disable_signals=True)
while not rospy.has_param('/robot_description'):
    pass

# Get robot description parameter and initialize model interface (with Pinocchio)
urdf = rospy.get_param('/robot_description')
model = xbi.ModelInterface2(urdf)



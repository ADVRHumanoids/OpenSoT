import os

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory
from xbot2_interface import pyxbot2_interface as xbi
import time
from pyopensot.tasks.velocity import Cartesian
from scipy.spatial.transform import Rotation as R
import numpy as np
from sensor_msgs.msg import JointState
import subprocess

from random import seed
from random import random

class ros2_node(Node):
    def __init__(self):
        super().__init__('franka_panda_ik')
        self.get_logger().info("franka_panda_ik ID node has been started.")
        self.client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')

        request = GetParameters.Request()
        request.names = ['robot_description']

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        self.urdf = None
        if future.result() is not None:
            values = future.result().values
            for val in values:
                self.urdf = val.string_value
        else:
            self.get_logger().error('Failed to call service')

        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

    def publish(self, joint_state_msg):
        self.joint_state_publisher.publish(joint_state_msg)

# seed random number generator
seed(1)
# generate random numbers between 0-1

def generate_random_delta(min, max):
    val = np.zeros(3)
    for i in range(3):
        val[i] = random()
    return min + (val * (max - min))

# Check for franka_cartesio_condif package
package_path = None
try:
    package_path = get_package_share_directory('franka_cartesio_config')
    print(f"Package path: {package_path}")
except:
    print("To run this example is needed the franka_cartesio_config package that can be download here: https://github.com/EnricoMingo/franka_cartesio_config")

roslaunch = subprocess.Popen(['ros2', 'launch', 'franka_cartesio_config', 'fp3.launch'], stdout=subprocess.PIPE, shell=False)
rviz_file_path = package_path + "/rviz/panda.rviz"
rviz = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '-d', f'{rviz_file_path}'], stdout=subprocess.PIPE, shell=False)

# Initiliaze node and wait for robot_description parameter
rclpy.init()
node = ros2_node()
model = xbi.ModelInterface2(node.urdf)

# Set a homing configuration
q = [0., -0.7, 0., -2.1, 0., 1.4, 0.]
model.setJointPosition(q)
model.update()


# Create a Cartesian task at frame panda_link7, set lambda gain
c = Cartesian("Cartesian", model, "fp3_link8", "world")
c.setLambda(1.)


# Get the actual reference for postural and Cartesian task
pose_ref, vel_ref = c.getReference()
dp = generate_random_delta(-0.2, 0.2)
pose_ref.translation += dp
pose_ref.linear = R.random().as_matrix()

print(f"pose_ref: {pose_ref}")

# IK loop
dt = 1./1000.


msg = JointState()
msg.name = model.getJointNames()
t = 0.
gamma = 0.1
epsilon = 1e-6


iter = 0
max_iter=10000
try:
    while rclpy.ok():
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
        msg.header.stamp = node.get_clock().now().to_msg()

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

        node.publish(msg)
        time.sleep(dt)
except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")
    roslaunch.kill()
    rviz.kill()
    node.destroy_node()

if rclpy.ok():
    rclpy.shutdown()
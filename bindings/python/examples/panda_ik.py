import os

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory
from xbot2_interface import pyxbot2_interface as xbi
from pyopensot.tasks.velocity import Postural, Cartesian, Manipulability, MinimumEffort
from pyopensot.constraints.velocity import JointLimits, VelocityLimits
import pyopensot as pysot
import numpy as np
from sensor_msgs.msg import JointState
import subprocess
import time

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
#
# Get Joint Limits and Velocity Limits, define dt
qmin, qmax = model.getJointLimits()
qlims = JointLimits(model, qmax, qmin)
#
dqmax = model.getVelocityLimits()
dt = 1./100.
dqlims = VelocityLimits(model, dqmax, dt)
#
# Create postural task (it is created at the q configuration previously set
p = Postural(model)
#
# Create a Cartesian task at frame panda_link7, set lambda gain
c = Cartesian("Cartesian", model, "fp3_link8", "world")
c.setLambda(0.1)
#
# Retrieve actual pose of the frame to be used as reference
ref = c.getActualPose().copy()
#
# Create the stack:
# 1st priority Cartesian position
# 2nd priority Cartesian orientation
# 3rd priority postural
s = ( (c%[0,1,2]) / (c%[3,4,5]) / p) << qlims
s<<dqlims
s.update()
#
# Get the actual reference for postural and Cartesian task
qref, dqref = p.getReference()
print(f"qref: {qref}")
print(f"dqref: {dqref}")
pose_ref, vel_ref = c.getReference()
print(f"pose_ref: {pose_ref}")
print(f"vel_ref: {vel_ref}")
#
# Creates iHQP solver with stack (using qpOASES as backend)
#
solver = pysot.iHQP(s)
#try:
#    import pyopensot_hcod
#    solver = pyopensot_hcod.HCOD(s, 1e-3)
#except ImportError:
#    raise ImportError('hcod solver not found. If you want to try it please compile with `-DOPENSOT_SOTH_FRONT_END=ON` option.')

#
msg = JointState()
msg.name = model.getJointNames()
# IK loop
t = 0.
alpha = 0.01
try:
    while rclpy.ok():
        # Update actual position in the model
        model.setJointPosition(q)
        model.update()
#
        # Compute new reference for Cartesian task
        pose_ref.translation[0] += alpha * np.sin(2.*3.1415 * t)
        pose_ref.translation[1] += alpha * np.cos(2.*3.1415 * t)
        t = t + alpha
        if t > 1.:
            t = 0
        c.setReference(pose_ref)
#
        # Update Stack
        s.update()
#
        # Solve
        dq = solver.solve()
        q += dq # Is fixed base so we can simply sum the result
#
        # Publish joint states
        msg.position = q
        msg.header.stamp = node.get_clock().now().to_msg()

        node.publish(msg)

        time.sleep(dt)
#
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
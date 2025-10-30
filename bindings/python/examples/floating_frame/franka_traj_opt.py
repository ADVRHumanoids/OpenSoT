from pyopensot.oc import *
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory
from xbot2_interface import pyxbot2_interface as xbi
from pyopensot import AffineHelper, OptvarHelper, GenericTask, Task, AffineTask, AffineConstraint
from pyopensot.tasks.velocity import Cartesian
from pyopensot.constraints.velocity import JointLimits
import pyopensot as pysot
import numpy as np
from sensor_msgs.msg import JointState
import subprocess
import time
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import PoseStamped, Point
from scipy.spatial.transform import Rotation as R
import unittest
import os
from utils import eul


np.set_printoptions(linewidth=np.inf)
class ros2_node(Node):
    def __init__(self):
        super().__init__('franka_panda_trajectory')
        self.get_logger().info("franka_panda_trajectory node has been started.")
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

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        self.marker_pose = PoseStamped()

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

Ns = 20 # number of nodes
tf = 2. # final time
dt = tf/Ns


model = xbi.ModelInterface2(node.urdf)
q_val = np.array([0., -0.7, 0., -2.1, 0., 1.4, 0.])
qdot_val = np.array([0., 0., 0., 0., 0., 0., 0.])
qddot_val = np.array([0., 0., 0., 0., 0., 0., 0.])

model.setJointPosition(q_val)
model.update()
T = model.getPose("fp3_link8")

"""
This set of variables describe the state and control inputs for the (NLP) OCP.
"""
vars = list()
# x
vars.append(("q", model.nq))
vars.append(("qdot", model.nv))
# u
vars.append(("qddot", model.nv))

variables = OptvarHelper(vars)
q = variables.getVariable("q")
qdot = variables.getVariable("qdot")
qddot = variables.getVariable("qddot")
print(f"variables.getSize(): {variables.getSize()}")

"""
This set of variables describe the state and control inputs for the internal liearized QP which acts in the tangent space.
In this particular case both sets of variables have the same size, but in general they could be different.
"""
dvars = list()
# dx
dvars.append(("dq", model.nv))
dvars.append(("dqdot", model.nv))
# du
dvars.append(("dqddot", model.nv))

dvariables = OptvarHelper(dvars)
dq = dvariables.getVariable("dq")
dqdot = dvariables.getVariable("dqdot")
dqddot = dvariables.getVariable("dqddot")

print(f"dvariables.getSize(): {dvariables.getSize()}")



class min_var(Task):
    """
    min_var consider the following function: F(var) = var - ref
    The dvariable is included to carry the information related to the size of the derivative of var
    """
    def __init__(self, name, variable, dvariable):
        super().__init__(name, variable.getInputSize())
        self.variable = variable
        self.dvariable = dvariable
        self.ref = 0. * self.variable.getq()
        self._W = np.eye(dvariable.getOutputSize())

    def _update(self):
        self.variable.update()
        self.dvariable.update()
        self.lin =  self.dvariable + (self.variable.getValue() - self.ref)
        self._A = self.lin.getM()
        self._b = -self.lin.getq()

    def setReference(self, ref):
        self.ref = ref

    @classmethod
    def create(cls, name, variable, dvariable):
        obj = cls(name, variable, dvariable)
        obj.update()
        return obj

class dynamics_derivative(Task):
    """
    This carries the derivative of the linear dynamics computed from euler.
    """
    def __init__(self, name, df):
        super().__init__(name, df.getInputSize())
        self.df = df
        self._W = np.eye(df.getOutputSize())

    def _update(self):
        self.lin = self.df
        self._A = self.lin.getM()
        self._b = -self.lin.getq()

    @classmethod
    def create(cls, name, df):
        obj = cls(name, df)
        obj.update()
        return obj

def euler(x, xdot, dt):
    return x + dt * xdot  # x1 = x0 + dt * xdot0


x = AffineHelper.pile(q, qdot)
xdot = AffineHelper.pile(qdot, qddot)

dx = AffineHelper.pile(dq, dqdot)
dxdot = AffineHelper.pile(dqdot, dqddot)

x0 = list()
for i in range(Ns+1):
    x0.append(np.concatenate((q_val, qdot_val)))

u0 = list()
for i in range(Ns):
    u0.append(qddot_val)

print(f"x0[0]: {x0[0]}")

ocp = OCP()
dd = list()
const = list()
for i in range(Ns):
    stage = Stage()
    """ First we include information related to the state space """
    stage.state_space = CompositeSpace([VectorSpace(model.nq), VectorSpace(model.nv)])

    """ We include both state variables and dvariables """
    stage.x = x
    stage.xdot = xdot
    stage.dx = dx

    """ We include both control variables and dvariables """
    stage.u = qddot
    stage.du = dqddot

    """ We include q and qdot defined for the state variables """
    stage.q = q
    stage.v = qdot
    stage.a = qddot

    stage.model = xbi.ModelInterface2(node.urdf)

    ocp.addStage(stage)



""" Last stage (Ns) does not have dynamics and control variables/dvariables """
stage = Stage()
stage.model = xbi.ModelInterface2(node.urdf)
stage.x = x
stage.xdot = xdot
stage.dx = dx
stage.state_space = CompositeSpace([VectorSpace(model.nq), VectorSpace(model.nq)])
stage.q = q
stage.v = qdot
ocp.addStage(stage)


ocp.update(x0, u0)


for i in range(Ns):
    df = pysot.oc.EulerVector(stage.model, dx, dxdot, ocp.stage(i).x, ocp.stage(i).xdot, ocp.stage(i+1).x, dt)
    dd.append(df)
    ocp.stage(i).dynamics_derivative = df

ocp.update(x0, u0)
print(f"ocp.getNumberOfNodes(): {ocp.getNumberOfNodes()}")


minus = list()
for i in range(Ns):
    minu = min_var.create(f"minu{i}", ocp.stage(i).u, ocp.stage(i).du)
    minu.setWeight(1e-3 * np.eye(model.nv))
    minus.append(minu)
    ocp.stage(i).stack = pysot.AutoStack(minu)

    # tau_min
    tau_lim = DynamicsConstraint(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
    const.append(tau_lim)
    ocp.stage(i).stack << tau_lim


# set goal at final state
minvel = min_var.create(f"minvel", ocp.stage(Ns).x[model.nq:], dvariables.getVariable("dqdot"))
minvel.setWeight(1e-3 * np.eye(model.nv))

cartesian_task = pysot.oc.SE3Task("Cartesian", ocp.stage(Ns).model, dvariables.getVariable("dq"), "fp3_link8")
cartesian_task.setWeight(1e3 * np.eye(6))
ocp.stage(Ns).stack = pysot.AutoStack(cartesian_task)


ocp.update(x0, u0)

# joint limits
qlims = list()
for i in range(Ns+1):
    qmin, qmax = model.getJointLimits()
    qlims_i = JointLimits(ocp.stage(i).model, qmax, qmin)
    qlims.append(qlims_i)
    ocp.stage(i).stack << AffineConstraint.toAffine(qlims[-1], dvariables.getVariable("dq"))


print("Initing solver...")
solver = pysot.swSQP(ocp)
solver.getOptions().max_iters = 100
solver.getOptions().verbose = True
solver.getOptions().line_search_strategy = 1
solver.getOptions().min_abs_delta_solution = 1e-3
solver.init()
print(f"{solver.getOptions().print()}")
# print("...solver inited!")

pose_ref = cartesian_task.getReference().copy()
# out of task space
pose_ref.translation[0] += 0.5

# joint lims test
# pose_ref.translation[0] -= 0.4
# pose_ref.translation[2] -= 0.4
# pose_ref.translation[1] += 0.2
# try orientatiion task


cartesian_task.setReference(pose_ref)

ocp.update(x0, u0)
success = solver.solve(x0, u0)


x0 = solver.getStateSolution()
u0 = solver.getControlSolution()

state = np.concatenate((q_val,qdot_val))
space = CompositeSpace([VectorSpace(model.nq), VectorSpace(model.nv)])
dt_sim = 0.0001
msg = JointState()
msg.name = model.getJointNames()
try:
    while rclpy.ok():
        
        input()
        for x in x0:
            msg.position = x[:model.nq].tolist()
            msg.header.stamp = node.get_clock().now().to_msg()
            node.publish(msg)
            time.sleep(dt)

        node.publish(msg)
        rclpy.spin_once(node, timeout_sec=dt_sim)

except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
finally:
    print("Stopping the node.")
    roslaunch.kill()
    rviz.kill()
    node.destroy_node()

if rclpy.ok():
    rclpy.shutdown()

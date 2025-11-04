from pyopensot.oc import *
import rclpy
from rclpy.node import Node
from xbot2_interface import pyxbot2_interface as xbi
from pyopensot import AffineHelper, OptvarHelper, GenericTask, Task, AffineTask, AffineConstraint
from pyopensot.tasks.velocity import Postural
from pyopensot.constraints.velocity import JointLimits
import pyopensot as pysot
import numpy as np
from sensor_msgs.msg import JointState
import subprocess
import time
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import unittest
import os
from utils import *
from ttictoc import tic, toc 


np.set_printoptions(linewidth=np.inf)

class ros2_node(Node):
    def __init__(self):
        name = "mynode"
        super().__init__(name)
        self.get_logger().info(f"{name} node has been started.")

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_impedance/joints_desired', 10)

        self.robot_description_subscriber = self.create_subscription(
            String,
            '/robot_description_no_hand',
            self.listener_callback,
            10)


        self.ee_pose_subscriber = self.create_subscription(
            PoseStamped,             # message type
            '/OpenSoT_MPC/target_pose',      # topic name
            self.pose_callback,      # callback function
            10                       # QoS (queue size)
        )
        self.get_logger().info('PoseSubscriber node has been started.')


        self.joint_states_subsriber = self.create_subscription(
            JointState,             # message type
            '/joint_states',      # topic name
            self.joint_states_callback,      # callback function
            10                       # QoS (queue size)
        )
        self.get_logger().info('JointSubscriber node has been started.')


        self.urdf=None
        self.pose_ref = None
        self.state = None
        while self.urdf is None or self.state is None:
            rclpy.spin_once(self)
        self.get_logger().info(f"{name} initialization complete")

    def pose_callback(self, msg: PoseStamped):
        # self.get_logger().info('PoseSubscriber node has been started.')

        self.pose_ref.translation[0] = msg.pose.position.x
        self.pose_ref.translation[1] = msg.pose.position.y
        self.pose_ref.translation[2] = msg.pose.position.z
        quat = [msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w]
        self.pose_ref.linear = R.from_quat(quat).as_matrix()

    def joint_states_callback(self, msg: JointState):
        self.state = np.concatenate((msg.position , msg.velocity))

    def listener_callback(self, msg):
        self.get_logger().info("URDF readed")
        self.urdf = msg.data

    def publish(self, joint_state_msg:JointState):
        # return
        self.joint_state_publisher.publish(joint_state_msg)


# Initiliaze node and wait for robot_description parameter
rclpy.init()
node = ros2_node()

# rviz_file_path = "/home/forest_ws/code/OpenSoT/bindings/python/examples/floating_frame/franka_mpc_real.rviz"
# rviz = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '-d', f'{rviz_file_path}'], stdout=subprocess.PIPE, shell=False)


Ns = 10 # number of nodes
tf = 0.5 # final time
dt = tf/Ns

model = xbi.ModelInterface2(node.urdf)
print(model.getJointNames())


# q_val = np.array([0., -0.7, 0., -2.1, 0., 1.4, 0.])
# qdot_val = np.array([0., 0., 0., 0., 0., 0., 0.])
# qddot_val = np.array([0., 0., 0., 0., 0., 0., 0.])

model.setJointPosition(node.state[:model.nq].copy())
model.update()
# T = model.getPose("panda_link8")

q_val = node.state[:model.nq].copy()
qdot_val = node.state[model.nq:].copy()
qddot_val = np.zeros(model.nv)


vars = list()
vars.append(("q", model.nq))
vars.append(("qdot", model.nv))
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


    """ """
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

print(f"ocp.getNumberOfNodes(): {ocp.getNumberOfNodes()}")
utest = unittest.TestCase()
utest.assertTrue(ocp.getNumberOfNodes() == Ns+1)


minus = list()
for i in range(Ns):
    minu = min_var.create(f"minu{i}", ocp.stage(i).u, ocp.stage(i).du)
    minu.setWeight(1e-3 * np.eye(model.nv))
    minus.append(minu)

    # postural = Postural(ocp.stage(i).model)
    # postural.setWeight(1e0 * np.eye(model.nv))
    # postural.setReference(q_val.copy())
    # minus.append(postural)

    ocp.stage(i).stack = pysot.AutoStack(minu) # + AffineTask.toAffine(postural, dvariables.getVariable("dq")))

    # tau_min
    tau_lim = DynamicsConstraint(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
    const.append(tau_lim)
    ocp.stage(i).stack << tau_lim


# set goal at final state
minvel = min_var.create(f"minvel", ocp.stage(Ns).x[model.nq:], dvariables.getVariable("dqdot"))
minvel.setWeight(1e3 * np.eye(model.nv))

cartesian_task = pysot.oc.SE3Task("Cartesian", ocp.stage(Ns).model, dvariables.getVariable("dq"), "panda_link8")
cartesian_task.setWeight(1e6 * np.eye(6))
ocp.stage(Ns).stack = pysot.AutoStack(cartesian_task + minvel)

T = cartesian_task.getReference()
node.pose_ref = T.copy()


ocp.update(x0, u0)
#
print("ocp updated!")

#joint limits
qlims = list()
for i in range(Ns+1):
    qmin, qmax = model.getJointLimits()
    qlims_i = JointLimits(ocp.stage(i).model, qmax, qmin)
    qlims.append(qlims_i)
    ocp.stage(i).stack = ocp.stage(i).stack << AffineConstraint.toAffine(qlims[-1], dvariables.getVariable("dq"))



print("Initing solver...")
solver = pysot.swSQP(ocp)
solver.getOptions().max_iters = 10
solver.getOptions().verbose = False
solver.getOptions().line_search_strategy = 0
solver.getOptions().beta = 1e-2
solver.getOptions().min_abs_delta_solution = 1e-3
solver.init()
print(f"{solver.getOptions().print()}")
print("...solver inited!")

dt_sim = 0.0001

# state = np.concatenate((q_val,qdot_val))
state = node.state.copy()
space = CompositeSpace([VectorSpace(model.nq), VectorSpace(model.nv)])


cartesian_task.setReference(node.pose_ref.copy())

ocp.update(x0, u0)
success = solver.solve(x0, u0)

x0 = solver.getStateSolution()
u0 = solver.getControlSolution()




print("-"*100)
# b = toc()
msg = JointState()
msg.name = model.getJointNames()
# print(msg.name)
try:
    while rclpy.ok():
        cartesian_task.setReference(node.pose_ref.copy())

        # x0[0] = node.state
        # tic()
        success = solver.solve(x0, u0)
        # b = toc()
        # print(b)

        x0 = solver.getStateSolution()
        u0 = solver.getControlSolution()


        # state = space.plus(state, np.concatenate((state[model.nq:], u0[0]))*dt)


        msg.position = x0[1][:model.nq].tolist()
        msg.velocity = x0[1][model.nq:].tolist()

        # print(msg.position)
        # print(msg.velocity)

        
        for i in range(len(x0)-1):
            x0[i] = x0[i+1]
        for i in range(len(u0)-1):
            u0[i] = u0[i+1]    
        u0[-1] = u0[-1]*0.
        
        # msg.header.stamp = node.get_clock().now().to_msg()
        node.publish(msg)

        rclpy.spin_once(node, timeout_sec=0.0)


except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
finally:
    print("Stopping the node.")
    node.destroy_node()

if rclpy.ok():
    rclpy.shutdown()

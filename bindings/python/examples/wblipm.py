import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
from pyopensot import AffineHelper, OptvarHelper, GenericTask, AggregatedTask
import pyopensot as pysot
from rclpy.node import Node
from pyopensot.tasks.acceleration import Cartesian, CoM, Postural, AngularMomentum
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
import pathlib

from xbot2_interface import pyxbot2_interface as xbi
import subprocess
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, WrenchStamped
from tf2_ros import TransformBroadcaster
import tictoc
import time

class ros2_node(Node):
    def __init__(self):
        super().__init__('g1_wblipm')
        self.get_logger().info("g1 ID node has been started.")

        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.base_link_broadcaster = TransformBroadcaster(self)

        self.joint_msg = JointState()
        self.w_T_b = TransformStamped()

    def publish(self, q):
        t = node.get_clock().now().to_msg()

        self.joint_msg.position = q[7::]
        self.joint_msg.header.stamp = t

        self.w_T_b.header.stamp = t
        self.w_T_b.transform.translation.x = q[0]
        self.w_T_b.transform.translation.y = q[1]
        self.w_T_b.transform.translation.z = q[2]
        self.w_T_b.transform.rotation.x = q[3]
        self.w_T_b.transform.rotation.y = q[4]
        self.w_T_b.transform.rotation.z = q[5]
        self.w_T_b.transform.rotation.w = q[6]

        self.joint_state_publisher.publish(self.joint_msg)
        self.base_link_broadcaster.sendTransform(self.w_T_b)

roslaunch = subprocess.Popen(['ros2', 'launch', 'hurobots', 'g1.launch'], stdout=subprocess.PIPE, shell=False)

package_path = get_package_share_directory('hurobots')
urdf_path = pathlib.Path(package_path + "/description_files/urdf/g1_29dof.urdf")
urdf_string = urdf_path.read_text()

model = xbi.ModelInterface2(urdf_string)

q = [ 0., 0., 0., 0., 0., 0., 1., # base
       -0.1, 0.,  0., #hips
        0.432, #knee
        -0.317, 0., # ankles
        -0.1, 0.,  0., #hips
        0.432, #knee
        -0.317, 0., #ankles
        0., 0., 0., # waist
        0.3,  0.25, 0., 1.,  0.15,  0., 0., # arm
        0.3, -0.25,  0., 1., 0.15,  0.,  0.] # arm

vel = np.zeros(model.nv)

model.setJointPosition(q)
model.setJointVelocity(vel)
model.update()

w_T_f = model.getPose("left_foot_point_contact")
w_T_bl = model.getPose("pelvis")

f_T_bl = w_T_f.inverse() * w_T_bl
q[0:3] = f_T_bl.translation
q[3:7] = R.from_matrix(f_T_bl.linear).as_quat()
model.setJointPosition(q)
model.setJointVelocity(vel)
model.update()

rclpy.init()
node = ros2_node()



node.joint_msg.name = model.getJointNames()[1::]
node.w_T_b.header.frame_id = "world"
node.w_T_b.child_frame_id = "pelvis"


def plot_trajectory(Ns, x_value, u_value, zmp_refs, dt):
    plt.figure(figsize=(8, 4))
    plt.plot(np.arange(Ns + 1) * dt, x_value[1, :], label='$r_y$ (CoM position)')
    plt.plot(np.arange(Ns - 1) * dt, u_value[1, :], label='$z_y$ (ZMP position)')
    #plt.plot(np.arange(Ns) * dt, zmp_refs[1, :], label='$z_y$ reference')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
def zmp_pattern(ns, offset_y=0.):
    zref = np.zeros((2, ns))
    for i in range(ns):
        zref[:, i] = np.zeros((2, 1)).flatten()
        if i < 10:
            zref[:, i] = np.array([0.0, offset_y])
        if i >= 10 and i < 20:
            zref[:, i] = np.array([0.0, offset_y + 0.1])
        elif i >= 20 and i < 30:
            zref[:, i] = np.array([0.0, offset_y - 0.1])
        elif i >= 30:
            zref[:, i] = np.array([0.0, offset_y + 0.])
    return zref

def lipm(r, z, h):
    w = np.sqrt(9.81 / h)
    return w*w*(r - z)

def euler(x0, xdot0, x1, dt):
    return x1 - x0 - dt * xdot0 # x1 = x0 + dt * xdot0

def initial_state_constraint(x0, value):
    tmp = x0 + value
    return GenericTask("initial_state", tmp.getM(), tmp.getq())

def min_u(u, R=np.array([[1, 0], [0, 1]]), id = "zmp_tracking"):
    T = GenericTask(id, u.getM(), u.getq())
    T.setWeight(R)
    return T

def min_x(x, Q=np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])):
    T = GenericTask("min_x", x.getM(), x.getq())
    T.setWeight(Q)
    return T


nx = 4 # com position and velocity  [x, y, xdot, ydot]
nu = 2 # zmp position [zmp_x, zmp_y]

Ns = 20 # number of nodes
tf = 1.5 # final time

vars = list()
for i in range(Ns):
    vars.append((f"x{i}", nx))
    if i == 0:
        vars.append((f"u{i}", model.getNv())) # nidot
    else:
        vars.append((f"u{i}", nu))
vars.append((f"x{Ns}", nx))

variables = OptvarHelper(vars)
print(f"variables.getSize(): {variables.getSize()}")

print(f"COM: {model.getCOM()}")

h = model.getCOM()[2]
dt = tf/Ns
integration = list()

# Integrate full Model
x0 = variables.getVariable(f"x0")
u0 = variables.getVariable(f"u0")
x1 = variables.getVariable(f"x1")
r = x0[0:2]
rdot = x0[2:]
rddot = model.getCOMJacobian()[0:2, :] @ u0 + model.getCOMJdotTimesV()[0:2]
xdot0 = AffineHelper.pile(rdot, rddot)
EULER = euler(x0, xdot0, x1, dt)
integration_0_constraint = GenericTask(f"integration_0", EULER.getM(), EULER.getq())

# Integrate LIPM
for i in range(1, Ns):
    x0 = variables.getVariable(f"x{i}")
    u0 = variables.getVariable(f"u{i}")
    x1 = variables.getVariable(f"x{i + 1}")

    r = x0[0:2]
    rdot = x0[2:]

    rddot = lipm(r, u0, h)

    xdot0 = AffineHelper.pile(rdot, rddot)

    integration_ = euler(x0, xdot0, x1, dt)
    integration.append(GenericTask(f"integration_{i}", integration_.getM(), integration_.getq()))


integration_constraint = AggregatedTask(integration, variables.getSize()) + integration_0_constraint
#plt.spy(integration_constraint.getA(), markersize=5)
#plt.show()

initial_state = initial_state_constraint(variables.getVariable("x0"), np.hstack((model.getCOM()[0:2], model.getCOMVelocity()[0:2])))

zmp_tasks = list()
for i in range(1, Ns):
    zmp_tasks.append(min_u(variables.getVariable(f"u{i}"), R=1e3 * np.array([[1, 0], [0, 1]])))
zmp_tracking_task = AggregatedTask(zmp_tasks, variables.getSize())

x_tasks = list()
for i in range(Ns+1):
    Q = 1e-3 * np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    if i == Ns:
        Q = 1e6 * np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    x_tasks.append(min_x(variables.getVariable(f"x{i}"), Q=Q))
min_xdot_task = AggregatedTask(x_tasks, variables.getSize())


# contact constraints for the first stage
foot_frames = ["left_foot_point_contact", "right_foot_point_contact"]
contact_tasks = dict()
for foot_frame in foot_frames:
    contact_tasks[foot_frame] = Cartesian(foot_frame + "_kin", model, foot_frame, "world", variables.getVariable("u0"))

# com task on z for the first stage
com = CoM(model, variables.getVariable("u0"))

# postural task for the first stage
postural = Postural(model, variables.getVariable("u0"))

# angular momentum task for the first stage
amom = AngularMomentum(model, variables.getVariable("u0"))

# orientation of the pelvis for the first stage
pelvis = Cartesian("pelvis", model, "pelvis", "world", variables.getVariable("u0"))

# Create the stack
cost = min_xdot_task + zmp_tracking_task + min_u(variables.getVariable("u0"), R=1e-3 * np.eye(model.getNv(), model.getNv()), id="min_acc") + com[2] + 1e-3 * postural[7:] + 0.1 * amom + 0.1 * pelvis[3:]
for foot_frame in foot_frames:
    cost = cost + contact_tasks[foot_frame]

constraints = integration_constraint + initial_state

Ns_ref = 40
zmp_refs = zmp_pattern(Ns_ref, offset_y=model.getCOM()[1])
for i in range(Ns-1):
    zmp_tasks[i].setb(zmp_refs[:, i])
    zmp_tasks[i].update()



# 1. Trajectory Optimization
stack = pysot.AutoStack(cost) << constraints
stack.update()
solver = pysot.iHQP(stack)

w = solver.solve()

x_value = np.zeros((nx, Ns+1))
acc_value = np.zeros((model.getNv(), 1))
u_value = np.zeros((nu, Ns-1))

for i in range(Ns+1):
    x_value[:,i] = variables.getVariable(f"x{i}").getValue(w)

acc_value = variables.getVariable(f"u{0}").getValue(w)

for i in range(1, Ns):
    u_value[:,i-1] = variables.getVariable(f"u{i}").getValue(w)

# Plot
plot_trajectory(Ns, x_value, u_value, zmp_refs, dt)

# 2. MPC
# zeroing references
for i in range(Ns-1):
    zmp_tasks[i].setb(model.getCOM()[0:2])

stack.update()

# --- Prepare plot ---
ry = deque([model.getCOM()[1]]*100)
zy = deque([model.getCOM()[1]]*100)

plt.ion()
line1, = plt.plot(ry, label='$r_y$ (CoM position)')
line2, = plt.plot(zy, label='$z_y$ (ZMP position)')
plt.ylim([-0.5,0.5])
plt.show()


scroll = 0
# --- Main Loop ---
t = 0

x0 = np.hstack((model.getCOM()[0:2], model.getCOMVelocity()[0:2]))
dt_sim = 0.01
try:
    while rclpy.ok():
        ry.append(x0[1])
        ryplot = ry.popleft()
        #tictoc.tic()



        initial_state.setb(x0)

        # shift reference to left
        for j in range(1, Ns-1):
            zmp_tasks[j-1].setb(zmp_tasks[j].getb())
        zmp_tasks[Ns-2].setb(zmp_refs[:, t % Ns_ref])

        stack.update()

        w = solver.solve()

        x_value = np.zeros((nx, Ns + 1))
        acc_value = np.zeros((model.getNv(), 1))
        u_value = np.zeros((nu, Ns - 1))

        for i in range(Ns + 1):
            x_value[:, i] = variables.getVariable(f"x{i}").getValue(w)

        acc_value = variables.getVariable(f"u{0}").getValue(w)

        for i in range(1, Ns):
            u_value[:, i - 1] = variables.getVariable(f"u{i}").getValue(w)


        q = model.sum(q, vel * dt_sim + 0.5 * acc_value * dt_sim**2)  # we use the model sum to account for the floating-base
        vel += acc_value * dt_sim

        model.setJointPosition(q)
        model.setJointVelocity(vel)
        model.update()

        x0 = np.hstack((model.getCOM()[0:2], model.getCOMVelocity()[0:2]))

        #print(tictoc.toc())

        t += 1
        zy.append(u_value[1, 0])
        zyplot = zy.popleft()

        line1.set_ydata(ry)
        line2.set_ydata(zy)
        plt.draw()

        # --- Publish ---
        node.publish(q)

        #ch = input()

        plt.pause(dt_sim)
        time.sleep(dt_sim)


except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    roslaunch.kill()
    print("Stopping the node.")

if rclpy.ok():
   rclpy.shutdown()





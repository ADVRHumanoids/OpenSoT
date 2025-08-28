import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
from pyopensot import AffineHelper, OptvarHelper, GenericTask, AggregatedTask, Task
import pyopensot as pysot
from rclpy.node import Node
from pyopensot.tasks.acceleration import Cartesian, CoM, Postural, AngularMomentum
from collections import deque
from ttictoc import tic, toc

import rclpy
from ament_index_python.packages import get_package_share_directory
import pathlib

from xbot2_interface import pyxbot2_interface as xbi
import subprocess
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, WrenchStamped
from tf2_ros import TransformBroadcaster

class ros2_node(Node):
    def __init__(self):
        super().__init__('g1_wblipm')
        self.get_logger().info("g1 ID node has been started.")
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.base_link_broadcaster = TransformBroadcaster(self)
        self.joint_msg = JointState()
        self.w_T_b = TransformStamped()
        self.w_T_b.header.frame_id = "world"
        self.w_T_b.child_frame_id = "pelvis"

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

urdf_string = pathlib.Path(get_package_share_directory('hurobots') + "/description_files/urdf/g1_29dof.urdf").read_text()

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

def plot_trajectory(Ns, x_value, u_value, zmp_refs, dt):
    plt.figure(figsize=(8, 4))
    plt.plot(np.arange(Ns + 1) * dt, x_value[1, :], label='$r_y$ (CoM position)')
    plt.plot(np.arange(Ns) * dt, u_value[1, :], label='$z_y$ (ZMP position)')
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

nx = 4 # com position and velocity  [x, y, xdot, ydot]
nu = 2 # zmp position [zmp_x, zmp_y]

Ns = 20 # number of nodes
tf = 1.2 # final time
print(f"Ns: {Ns}, tf: {tf}")

vars = list()
vars.append((f"x0", nx))
vars.append((f"acc0", model.getNv()))
for i in range(Ns):
    vars.append((f"u{i}", nu))

variables = OptvarHelper(vars)
print(f"variables.getSize(): {variables.getSize()}")

class rddot(AffineHelper):
    def __init__(self, model, u0):
        super().__init__(model.getNv(), nu)
        self.model = model
        self.u0 = u0
        self.update()

    def update(self):
        self.rddot = self.model.getCOMJacobian()[0:2, :] @ self.u0 + self.model.getCOMJdotTimesV()[0:2]
        self.setM(self.rddot.getM())
        self.setq(self.rddot.getq())


rddot0 = rddot(model, variables.getVariable("acc0"))
rddot0.update()

print(f"COM: {model.getCOM()}")

h = model.getCOM()[2]
print(f"h: {h}")
dt = tf/Ns
print(f"dt: {dt}")

def lipm(r, z, h):
    w = np.sqrt(9.81 / h)
    return w*w*(r - z)

def euler(x, xdot, dt):
    return x + dt * xdot # x1 = x0 + dt * xdot0


class state(AffineHelper):
    def __init__(self, ns, variables, rddot0, h, dt):
        super().__init__()
        self.x = None
        self.rddot0 = rddot0
        self.h = h
        self.dt = dt
        self.variables = variables
        self.ns = ns

        self.xdot = list()

    def update(self):
        self.x = self.variables.getVariable("x0")
        if self.ns == 0:
            self._M = self.x.getM()
            self._q = self.x.getq()
        else:
            for i in range(self.ns):
                r = self.x[0:2]
                rdot = self.x[2:]
                if i == 0:
                    rddot = self.rddot0
                else:
                    rddot = lipm(r, self.variables.getVariable(f"u{i}"), self.h)

                if len(self.xdot) < i + 1:
                    xdot = AffineHelper.pile(rdot, rddot)
                    self.xdot.append(xdot)
                else:
                    xdot = self.xdot[i]
                    _M = xdot.getM()
                    _M[0:rdot.getM().shape[0], :] = rdot.getM()
                    _M[rdot.getM().shape[0]:, :] = rddot.getM()
                    _q = xdot.getq()
                    _q[0:rdot.getq().shape[0]] = rdot.getq()
                    _q[rdot.getq().shape[0]:] = rddot.getq()
                    self.xdot[i].setM(_M)
                    self.xdot[i].setq(_q)

                self.x = euler(self.x, self.xdot[i], self.dt)

            self._M = self.x.getM()
            self._q = self.x.getq()




xNs = state(Ns, variables, rddot0, h, dt)
xNs.update()
print(f"xNs: {xNs}")
min_rdot_final = GenericTask("min_rdot_final", xNs[2:])



class lipm_constraint(Task):
    def __init__(self, r0, rddot0, u0, h):
        super().__init__("lipm_constraint", r0.getInputSize())
        self.rddot0 = rddot0
        self.r0 = r0
        self.u0 = u0
        self.h = h

    def _update(self):
        constr = self.rddot0 - lipm(self.r0, self.u0, self.h)
        self._A = constr.getM()
        self._b = -constr.getq()

    @classmethod
    def create(cls, r0, rddot0, u0, h):
        obj = cls(r0, rddot0, u0, h)
        obj.update()
        return obj

lipmc = lipm_constraint.create(variables.getVariable("x0")[0:2], rddot0, variables.getVariable("u0"), h)


xinit = variables.getVariable("x0") - np.hstack((model.getCOM()[0:2], model.getCOMVelocity()[0:2]))
initial_state = GenericTask("initial_state", xinit)

zmp_tasks = list()
for i in range(Ns):
    min_ui = GenericTask("zmp_tracking", variables.getVariable(f"u{i}"))
    min_ui.setWeight(1e4 * np.array([[1, 0], [0, 1]]))
    zmp_tasks.append(min_ui)
zmp_tracking_task = AggregatedTask(zmp_tasks, variables.getSize())

# contact constraints for the first stage
foot_frames = ["left_foot_point_contact", "right_foot_point_contact"]
contact_tasks = dict()
for foot_frame in foot_frames:
    contact_tasks[foot_frame] = Cartesian(foot_frame + "_kin", model, foot_frame, "world", variables.getVariable("acc0"))

# com task on z for the first stage
com = CoM(model, variables.getVariable("acc0"))

# postural task for the first stage
postural = Postural(model, variables.getVariable("acc0"))

# angular momentum task for the first stage
amom = AngularMomentum(model, variables.getVariable("acc0"))

# orientation of the pelvis for the first stage
pelvis = Cartesian("pelvis", model, "pelvis", "world", variables.getVariable("acc0"))


# Create the stack
min_acc = GenericTask("min_acc", variables.getVariable("acc0"))
cost = 10.*min_rdot_final + zmp_tracking_task + com[2] + 1e-3 * min_acc + .1 * pelvis[3:] + 1. * postural[18:] + 1e-1 * amom
for foot_frame in foot_frames:
    cost = cost + 1. * contact_tasks[foot_frame]

lipmc = lipm_constraint.create(variables.getVariable("x0")[0:2], rddot0, variables.getVariable("u0"), h)
constraints = initial_state + lipmc

Ns_ref = 40
zmp_refs = zmp_pattern(Ns_ref, offset_y=model.getCOM()[1])
for i in range(Ns):
    zmp_tasks[i].setb(zmp_refs[:, i])
    zmp_tasks[i].update()



# 1. Trajectory Optimization
stack = pysot.AutoStack(cost) << constraints
stack.update()
solver = pysot.iHQP(stack)

tic()
w = solver.solve()
elapsed = toc()  # End timer and print elapsed time
print(f"Elapsed time: {elapsed:.3f} seconds")

x_value = np.zeros((nx, Ns+1))
u_value = np.zeros((nu, Ns))

for i in range(Ns):
    u_value[:, i] = variables.getVariable(f"u{i}").getValue(w)
    xi = state(i, variables, rddot0, h, dt)
    xi.update()
    x_value[:, i] = xi.getValue(w)
x_value[:, Ns] = xNs.getValue(w)


# Plot
plot_trajectory(Ns, x_value, u_value, zmp_refs, dt)

# 2. MPC
# zeroing references
for i in range(Ns):
    zmp_tasks[i].setb(model.getCOM()[0:2])

model.setJointPosition(q)
model.setJointVelocity(vel)
model.update()

rddot0.update()
xNs.update()

stack.update()


# --- Prepare plot ---
ry = deque([model.getCOM()[1]]*100)
zy = deque([model.getCOM()[1]]*100)
zry = deque([model.getCOM()[1]]*100)

plt.ion()
line1, = plt.plot(ry, label='$r_y$ (CoM position)')
line2, = plt.plot(zy, label='$z_y$ (ZMP position)')
line3, = plt.plot(zry, label='$zr_y$ (ZMP ref)')
plt.ylim([-0.5,0.5])
plt.show()

# --- Main Loop ---
t = 0
t_mpc = 0.

x0 = np.hstack((model.getCOM()[0:2], model.getCOMVelocity()[0:2]))
dt_sim = 0.02

xi = list()
for i in range(Ns):
    xi.append(state(i, variables, rddot0, h, dt))

try:
    while rclpy.ok():
        ry.append(x0[1])
        ryplot = ry.popleft()

        initial_state.setb(x0)

        # shift reference to left
        for j in range(1, Ns):
            zmp_tasks[j - 1].setb(zmp_tasks[j].getb())
        zmp_tasks[Ns - 1].setb(zmp_refs[:, t % Ns_ref])
        zry.append(zmp_tasks[0].getb()[1])
        zryplot = zry.popleft()

        tic()
        stack.update()

        w = solver.solve()
        t_mpc += toc()

        x_value = np.zeros((nx, Ns + 1))
        acc_value = np.zeros((model.getNv(), 1))
        u_value = np.zeros((nu, Ns))

        for i in range(Ns):
            xi[i].update()
            x_value[:, i] = xi[i].getValue(w)
        xNs.update()
        x_value[:, Ns] = xNs.getValue(w)

        acc_value = variables.getVariable("acc0").getValue(w)

        for i in range(Ns):
            u_value[:, i] = variables.getVariable(f"u{i}").getValue(w)

        q = model.sum(q, vel * dt_sim + 0.5 * acc_value * (dt_sim ** 2))  # we use the model sum to account for the floating-base
        vel += acc_value * dt_sim

        model.setJointPosition(q)
        model.setJointVelocity(vel)
        model.update()

        rddot0.update()
        xNs.update()

        x0 = np.hstack((model.getCOM()[0:2], model.getCOMVelocity()[0:2]))

        t += 1
        zy.append(u_value[1, 0])
        zyplot = zy.popleft()

        line1.set_ydata(ry)
        line2.set_ydata(zy)
        line3.set_ydata(zry)
        plt.draw()

        # --- Publish ---
        node.publish(q)

        # ch = input()

        plt.pause(dt_sim)
        # time.sleep(dt_sim)


        pass
except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    roslaunch.kill()
    print("Stopping the node.")
    print("Average mpt time: {:.3f} seconds".format(t_mpc / t))

if rclpy.ok():
   rclpy.shutdown()




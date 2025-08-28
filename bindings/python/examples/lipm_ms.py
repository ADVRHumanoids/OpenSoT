import matplotlib.pyplot as plt
import numpy as np
from pyopensot import AffineHelper, OptvarHelper, GenericTask, AggregatedTask
import pyopensot as pysot
import rclpy
from collections import deque
from ttictoc import tic, toc

def plot_trajectory(Ns, x_value, u_value, zmp_refs, dt):
    plt.figure(figsize=(8, 4))
    plt.plot(np.arange(Ns + 1) * dt, x_value[1, :], label='$r_y$ (CoM position)')
    plt.plot(np.arange(Ns) * dt, u_value[1, :], label='$z_y$ (ZMP position)')
    plt.plot(np.arange(Ns) * dt, zmp_refs[1, :], label='$z_y$ reference')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [m]')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

def zmp_pattern(ns):
    zref = np.zeros((2, ns))
    for i in range(ns):
        zref[:, i] = np.zeros((2, 1)).flatten()
        if i >= 10 and i < 20:
            zref[:, i] = np.array([0.0, 0.1])
        elif i >= 20 and i < 30:
            zref[:, i] = np.array([0.0, -0.1])
        elif i >= 30:
            zref[:, i] = np.array([0.0, 0.])
    return zref

def lipm(r, z, h):
    w = np.sqrt(9.81 / h)
    return w*w*(r - z)

def euler(x0, xdot0, x1, dt):
    return x1 - x0 - dt * xdot0 # x1 = x0 + dt * xdot0


rclpy.init()

nx = 4 # com position and velocity  [x, y, xdot, ydot]
nu = 2 # zmp position [zmp_x, zmp_y]

Ns = 40 # number of nodes
tf = 3.0 # final time

vars = list()
for i in range(Ns):
    vars.append((f"x{i}", nx))
    vars.append((f"u{i}", nu))
vars.append((f"x{Ns}", nx))

variables = OptvarHelper(vars)
print(f"variables.getSize(): {variables.getSize()}")

h = 0.83  # height of the CoM
dt = tf/Ns
integration = list()

for i in range(Ns):
    x0 = variables.getVariable(f"x{i}")
    u0 = variables.getVariable(f"u{i}")
    x1 = variables.getVariable(f"x{i+1}")

    r = x0[0:2]
    rdot = x0[2:]
    rddot = lipm(r, u0, h)

    xdot0 = AffineHelper.pile(rdot, rddot)

    integration_ = euler(x0, xdot0, x1, dt)
    integration.append(GenericTask(f"integration_{i}", integration_))


integration_constraint = AggregatedTask(integration, variables.getSize())
#plt.spy(integration_constraint.getA(), markersize=5)
#plt.show()

xinit = variables.getVariable("x0") - np.array([0., 0., 0, 0.])
initial_state = GenericTask("initial_state", xinit)

zmp_tasks = list()
for i in range(Ns):
    min_ui = GenericTask("zmp_tracking", variables.getVariable(f"u{i}"))
    min_ui.setWeight(1e1 * np.array([[1, 0], [0, 1]]))
    zmp_tasks.append(min_ui)
zmp_tracking_task = AggregatedTask(zmp_tasks, variables.getSize())

x_tasks = list()
for i in range(Ns+1):
    min_xi = GenericTask("min_x", variables.getVariable(f"x{i}"))
    Q = 1e-3 * np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    if i == Ns:
        Q = 1e6 * np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    min_xi.setWeight(Q)
    x_tasks.append(min_xi)
min_xdot_task = AggregatedTask(x_tasks, variables.getSize())

# Create the stack
cost = min_xdot_task + zmp_tracking_task
constraints = integration_constraint + initial_state

zmp_refs = zmp_pattern(Ns)
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
    x_value[:,i] = variables.getVariable(f"x{i}").getValue(w)
    u_value[:, i] = variables.getVariable(f"u{i}").getValue(w)
x_value[:,Ns] = variables.getVariable(f"x{Ns}").getValue(w)


# Plot
plot_trajectory(Ns, x_value, u_value, zmp_refs, dt)

# 2. MPC
# zeroing references
for i in range(Ns):
    zmp_tasks[i].setb(np.zeros((2, 1)))

stack.update()

# --- Prepare plot ---
ry = deque([0.]*100)
zy = deque([0.]*100)
zry = deque([0.]*100)

plt.ion()
line1, = plt.plot(ry, label='$r_y$ (CoM position)')
line2, = plt.plot(zy, label='$z_y$ (ZMP position)')
line3, = plt.plot(zry, label='$zr_y$ (ZMP ref)')
plt.ylim([-0.5, 0.5])
plt.show()
# --- Main Loop ---
t = 0

t_mpc = 0.

x0 = np.zeros((nx, 1))
try:
    while rclpy.ok():
        ry.append(x0[1,0])
        ryplot = ry.popleft()

        initial_state.setb(x0)

        # shift reference to left
        for j in range(1, Ns):
            zmp_tasks[j-1].setb(zmp_tasks[j].getb())
        zmp_tasks[Ns-1].setb(zmp_refs[:, t % Ns])
        zry.append(zmp_tasks[0].getb()[1])
        zryplot = zry.popleft()

        tic()
        stack.update()

        w = solver.solve()
        t_mpc += toc()

        for i in range(Ns):
            x_value[:, i] = variables.getVariable(f"x{i}").getValue(w)
            u_value[:, i] = variables.getVariable(f"u{i}").getValue(w)
        x_value[:, Ns] = variables.getVariable(f"x{Ns}").getValue(w)

        rdot = x_value[2:4, 0].flatten()
        rddot = lipm(x_value[0:2, 0], u_value[:, 0], h)

        xdot = np.hstack((rdot, rddot)).reshape((nx, 1))
        x0 += dt * xdot

        t += 1
        zy.append(u_value[1, 0])
        zyplot = zy.popleft()

        line1.set_ydata(ry)
        line2.set_ydata(zy)
        line3.set_ydata(zry)
        plt.draw()

        plt.pause(dt)

except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")
    print("Average mpt time: {:.3f} seconds".format(t_mpc / t))

if rclpy.ok():
   rclpy.shutdown()





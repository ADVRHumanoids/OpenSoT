import matplotlib.pyplot as plt
import numpy as np
from pyopensot import AffineHelper, OptvarHelper, GenericTask, AggregatedTask
import pyopensot as pysot
import pyopensot_hpipmoc as hpipmoc
from ttictoc import tic, toc
import unittest
from collections import deque

SHOW_PLOTS = False

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

np.set_printoptions(linewidth=np.inf)

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

def lipm(r, z, h):
    w = np.sqrt(9.81 / h)
    return w*w*(r - z)

def euler(x, xdot, dt):
    return x + dt * xdot # x1 = x0 + dt * xdot0

h = 0.83  # height of the CoM
dt = tf/Ns
integration = list()

for i in range(Ns):
    x0 = variables.getVariable(f"x{i}")
    u0 = variables.getVariable(f"u{i}")

    r = x0[0:2]
    rdot = x0[2:]
    rddot = lipm(r, u0, h)

    xdot0 = AffineHelper.pile(rdot, rddot)

    integration_ = euler(x0, xdot0, dt)
    integration.append(GenericTask(f"integration_{i}", integration_.getM(), -integration_.getq()))


print(f"integration[0].getA()\n: {integration[0].getA()}")
# to retrieve A and B for each stage:
print(f"integration[0].getA() @ variables.getVariable(x0).getM().T \n: {integration[0].getA() @ variables.getVariable("x0").getM().T}")
print(f"integration[0].getA() @ variables.getVariable(u0).getM().T \n: {integration[0].getA() @ variables.getVariable("u0").getM().T}")

solver = hpipmoc.hpipmOC(Ns)
for i in range(Ns):
    Ai = integration[i].getA() @ variables.getVariable(f"x{i}").getM().T
    Bi = integration[i].getA() @ variables.getVariable(f"u{i}").getM().T
    bi = np.zeros((nx, 1))
    solver.setStageDynamics(i, Ai, Bi, bi)

#xinit = variables.getVariable("x0") + np.array([0., 0., 0, 0.])
#initial_state = GenericTask("initial_state", xinit.getM(), -xinit.getq())

#print(f"initial_state.getA()\n: {initial_state.getA()}")
#print(f"initial_state.getA() @ variables.getVariable(x0).getM().T\n: {initial_state.getA() @ variables.getVariable("x0").getM().T}")

#id = [0, 1, 2, 3] # these are the indices of the first 4 state variables (x) at stage 0
#solver.setBoxConstraintsX(0, id, initial_state.getb(), initial_state.getb())

zmp_tasks = list()
for i in range(Ns):
    min_ui = GenericTask("zmp_tracking", variables.getVariable(f"u{i}").getM(), -variables.getVariable(f"u{i}").getq())
    min_ui.setWeight(1e4 * np.array([[1, 0], [0, 1]]))
    zmp_tasks.append(min_ui)

print(f"zmp_tasks[2].getA()\n: {zmp_tasks[2].getA()}")
print(f"zmp_tasks[2].getA() @ variables.getVariable(u2).getM().T\n: {zmp_tasks[2].getA() @ variables.getVariable("u2").getM().T}")

print(f"zmp_tasks[2].getb()\n: {zmp_tasks[2].getb()}")
print(f"zmp_tasks[2].getWeight()\n: {zmp_tasks[2].getWeight()}")

zmp_refs = zmp_pattern(Ns)
for i in range(Ns):
    zmp_tasks[i].setb(zmp_refs[:, i])
    zmp_tasks[i].update()

x_tasks = list()
for i in range(Ns+1):
    min_xi = GenericTask("min_x", variables.getVariable(f"x{i}").getM(), variables.getVariable(f"x{i}").getq())
    Q = 1e-3 * np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    if i == Ns:
        Q = 1e6 * np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    min_xi.setWeight(Q)
    x_tasks.append(min_xi)


for i in range(Ns):
    solver.setLSCost(i,
                     x_tasks[i].getA() @ variables.getVariable(f"x{i}").getM().T, x_tasks[i].getWeight(), x_tasks[i].getb(),
                     zmp_tasks[i].getA() @ variables.getVariable(f"u{i}").getM().T, zmp_tasks[i].getWeight(), zmp_tasks[i].getb())
solver.setLSCost(Ns, x_tasks[Ns].getA() @ variables.getVariable(f"x{Ns}").getM().T, x_tasks[Ns].getWeight(), x_tasks[Ns].getb())

tic()
success = solver.solve(np.array([0., 0., 0, 0.]))
elapsed = toc()  # End timer and print elapsed time
print(f"Elapsed time: {elapsed:.3f} seconds")

utest = unittest.TestCase()
utest.assertTrue(success)

if(success):
    print("HPIPM solved the problem!")
    solution = solver.getSolution()

    x_value = np.zeros((nx, Ns + 1))
    u_value = np.zeros((nu, Ns))

    for i in range(Ns):
        x_value[:, i] = solution[i].x
        u_value[:, i] = solution[i].u
    x_value[:, Ns] = solution[Ns].x

    # Plot
    if SHOW_PLOTS:
        plot_trajectory(Ns, x_value, u_value, zmp_refs, dt)
else:
    print("HPIPM could not solve the problem!")

# zeroing references
for i in range(Ns):
    zmp_tasks[i].setb(np.zeros((2, 1)))
    zmp_tasks[i].update()

# --- Prepare plot ---
x_value = np.zeros((nx, Ns + 1))
u_value = np.zeros((nu, Ns))

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
t_mpc = 0.

x0 = np.zeros((nx, 1))
t = 0
for k in range(100):
    ry.append(x0[1,0])
    ryplot = ry.popleft()

    #initial_state.setb(x0)
    #initial_state.update()

    # shift reference to left
    for j in range(1, Ns):
        zmp_tasks[j-1].setb(zmp_tasks[j].getb())
        zmp_tasks[j-1].update()
    zmp_tasks[Ns-1].setb(zmp_refs[:, t % Ns])
    zmp_tasks[Ns-1].update()
    zry.append(zmp_tasks[0].getb()[1])
    zryplot = zry.popleft()

    # update internal qp
    #solver.setBoxConstraintsX(0, id, initial_state.getb(), initial_state.getb())
    for i in range(Ns):
        solver.setLSCost(i,
                         x_tasks[i].getA() @ variables.getVariable(f"x{i}").getM().T, x_tasks[i].getWeight(), x_tasks[i].getb(),
                         zmp_tasks[i].getA() @ variables.getVariable(f"u{i}").getM().T, zmp_tasks[i].getWeight(), zmp_tasks[i].getb())
    solver.setLSCost(Ns, x_tasks[Ns].getA() @ variables.getVariable(f"x{Ns}").getM().T, x_tasks[Ns].getWeight(), x_tasks[Ns].getb())

    tic()
    success = solver.solve(x0)
    utest.assertTrue(success)
    t_mpc += toc()

    solution = solver.getSolution()
    for i in range(Ns):
        x_value[:, i] = solution[i].x
        u_value[:, i] = solution[i].u
    x_value[:, Ns] = solution[Ns].x

    utest.assertTrue(( np.linalg.norm(u_value[:,0] - zmp_tasks[0].getb(), ord=2) <= 1e-4).all())

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

    if SHOW_PLOTS:
        plt.draw()
        plt.pause(dt)

print("Average mpt time: {:.3f} seconds".format(t_mpc / t))
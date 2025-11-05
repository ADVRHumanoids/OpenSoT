import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from xbot2_interface import pyxbot2_interface as xbi
import numpy as np
import subprocess
import time
from scipy.spatial.transform import Rotation as R
from pyopensot import AffineHelper, OptvarHelper
from pyopensot.oc import *
from pyopensot.tasks.velocity import Postural

from utils import *

roslaunch = subprocess.Popen(['ros2', 'launch', '/home/forest_ws/code/OpenSoT/bindings/python/examples/ocp_examples/double_pendulum/pendulum.launch.py'], stdout=subprocess.PIPE, shell=False)
rviz_file_path = "/home/forest_ws/code/OpenSoT/bindings/python/examples/ocp_examples/double_pendulum/doub_pend.rviz"
rviz = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '-d', f'{rviz_file_path}'], stdout=subprocess.PIPE, shell=False)
rclpy.init()
rosnode = double_pendulum_node()


model = xbi.ModelInterface2(rosnode.urdf)
q_val = np.array([0., 0.])
qdot_val = np.array([0., 0.])
qddot_val = np.array([0., 0.])
rosnode.publish(model, q_val)
rclpy.spin_once(rosnode, timeout_sec=0.1)


vars = list()
# x
vars.append(("q", model.nq))
vars.append(("qdot", model.nv))
vars.append(("qddot", model.nv))

variables = OptvarHelper(vars)
q = variables.getVariable("q")
qdot = variables.getVariable("qdot")
qddot = variables.getVariable("qddot")


dvars = list()
dvars.append(("dq", model.nv))
dvars.append(("dqdot", model.nv))
dvars.append(("dqddot", model.nv))

dvariables = OptvarHelper(dvars)
dq = dvariables.getVariable("dq")
dqdot = dvariables.getVariable("dqdot")
dqddot = dvariables.getVariable("dqddot")

x = AffineHelper.pile(q, qdot)
xdot = AffineHelper.pile(qdot, qddot)

dx = AffineHelper.pile(dq, dqdot)
dxdot = AffineHelper.pile(dqdot, dqddot)


Ns = 100 # number of nodes
tf = 3. # final time
dt = tf/Ns

x0 = list()
for i in range(Ns+1):
    x0.append(np.concatenate((q_val, qdot_val)))

u0 = list()
for i in range(Ns):
    u0.append(qddot_val)

ocp = OCP()
dd = list()
const = list()
minus = list()
for i in range(Ns+1):
    stage = Stage()
    """ First we include information related to the state space """
    stage.state_space = CompositeSpace([VectorSpace(model.nq), VectorSpace(model.nv)])

    """ We include both state variables and dvariables """
    stage.x = x
    stage.xdot = xdot
    stage.dx = dx

    if i<Ns:
        """ We include both control variables and dvariables """
        stage.u = qddot
        stage.du = dqddot

    """ We include q and qdot defined for the state variables """
    stage.q = q
    stage.v = qdot
    stage.a = qddot

    stage.model = xbi.ModelInterface2(rosnode.urdf)

    ocp.addStage(stage)

ocp.update(x0, u0)

mintaus = []
for i in range(Ns):
    df = EulerVector(stage.model, dx, dxdot, ocp.stage(i).x, ocp.stage(i).xdot, ocp.stage(i+1).x, dt)
    dd.append(df)
    ocp.stage(i).dynamics_derivative = df


    minu = min_var.create(f"minu{i}", ocp.stage(i).u, ocp.stage(i).du)
    minu.setWeight(1e-9 * np.eye(model.nv))
    minus.append(minu)
    

    mintau = TorquesTask(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
    mintau.setWeight(0 * np.eye(model.nv))
    mintaus.append(mintau)
    ocp.stage(i).stack = pysot.AutoStack(minu+mintau)

    # tau_min
    tau_lim = DynamicsConstraint(ocp.stage(i).model, ocp.stage(i).dx, ocp.stage(i).du)
    tau_lim.setTorqueLimit([10., 0.])
    const.append(tau_lim)
    ocp.stage(i).stack << tau_lim

minvel = min_var.create(f"minvel", ocp.stage(Ns).x[model.nq:], dvariables.getVariable("dqdot"))
minvel.setWeight(1e0 * np.eye(model.nv))

postural = Postural(ocp.stage(Ns).model)
postural.setWeight(1e3 * np.eye(model.nv))
postural.setReference([np.pi, 0.])
ocp.stage(Ns).stack = pysot.AutoStack(minvel + AffineTask.toAffine(postural, dvariables.getVariable("dq")))

ocp.update(x0, u0)

print("Initing solver...")
solver = pysot.swSQP(ocp)
solver.getOptions().max_iters = 1000
solver.getOptions().verbose = True
solver.getOptions().line_search_strategy = 1
solver.getOptions().beta = 1e-2
solver.getOptions().min_abs_delta_solution = 1e-3
solver.init()
print(f"{solver.getOptions().print()}")
print("...solver inited!")



ocp.update(x0, u0)
success = solver.solve(x0, u0)

x0 = solver.getStateSolution()
u0 = solver.getControlSolution()


try:
    t= 0.
    while rclpy.ok():
        input()

        x = x0[0]
        for i in range(len(x0)):
            x = x0[i]
            q_val = x.tolist()[:model.nv]
            # if i<Ns: print(-mintaus[i].getb())
            rosnode.publish(model, q_val)
            time.sleep(dt)

        rosnode.publish(model, q_val)

        rclpy.spin_once(rosnode, timeout_sec=0.0)

        # time.sleep(0.001)
        

except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")
    rviz.kill()
    roslaunch.kill()
    rosnode.destroy_node()

if rclpy.ok():
    rclpy.shutdown()

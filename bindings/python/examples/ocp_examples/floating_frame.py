import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from xbot2_interface import pyxbot2_interface as xbi
import pyopensot as pysot
import numpy as np
from std_msgs.msg import String 
from sensor_msgs.msg import JointState
import subprocess
import time
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import PoseStamped, Point
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from pyopensot import AffineHelper, OptvarHelper, GenericTask, Task, AffineTask, AffineConstraint
import math

from utils import *

rviz_file_path = "/home/forest_ws/code/OpenSoT/bindings/python/examples/ocp_examples/floating_frame/floating_frame.rviz"
rviz = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '-d', f'{rviz_file_path}'], stdout=subprocess.PIPE, shell=False)

rclpy.init()
rosnode = floating_frame_node()

model = xbi.ModelInterface2(rosnode.urdf)

Ns = 20 # number of nodes
tf = 1.0 # final time
dt = tf/Ns


#q_val = np.array([0., 0., 0., 0., 0., 0., 1.])
q_val = random_pose(-2., 2.)
v_val = np.array([0., 0., 0., 0., 0., 0.])

#q_final = np.array([0., 0., 0., 0., 0., 0., 1.])
q_final = random_pose(-2,2)

rosnode.publish_start(q_val)
rosnode.publish_goal(q_final)

vars = list()
vars.append(("q", model.nq))
vars.append(("qdot", model.nv))

variables = OptvarHelper(vars)
q = variables.getVariable("q")
qdot = variables.getVariable("qdot")


print(f"variables.getSize(): {variables.getSize()}")



dvars = list()
dvars.append(("dq", model.nv))
dvars.append(("dqdot", model.nv))

dvariables = OptvarHelper(dvars)
dq = dvariables.getVariable("dq")
dqdot = dvariables.getVariable("dqdot")

print(f"dvariables.getSize(): {dvariables.getSize()}")


x = q
xdot = qdot

dx = dq
dxdot = dqdot



x0 = list()
for i in range(Ns+1):
    x0.append(q_val)

u0 = list()
for i in range(Ns):
    u0.append(v_val)


ocp = pysot.oc.OCP()
dd = list()
for i in range(Ns):
    stage = pysot.oc.Stage()

    stage.model = xbi.ModelInterface2(rosnode.urdf)
    stage.state_space = pysot.oc.SE3Space()

    stage.x = x
    stage.dx = dx

    stage.u = qdot
    stage.du = dqdot

    stage.q = q
    stage.v = qdot

    ocp.addStage(stage)

#final stage definition
stage = pysot.oc.Stage()
stage.model = xbi.ModelInterface2(rosnode.urdf)
stage.state_space = pysot.oc.SE3Space()
stage.x = x
stage.dx = dx
stage.q = q
stage.v = qdot
ocp.addStage(stage)

ocp.update(x0, u0)

for i in range(Ns):
    df = pysot.oc.EulerSE3(stage.model, dq, dqdot, ocp.stage(i).x, ocp.stage(i).u, ocp.stage(i+1).x, dt)
    dd.append(df)
    ocp.stage(i).dynamics_derivative = df

ocp.update(x0, u0)

print(f"ocp.getNumberOfNodes(): {ocp.getNumberOfNodes()}")



minus = list()
for i in range(Ns):
    minu = min_var.create(f"minu{i}", ocp.stage(i).u, ocp.stage(i).du)
    minu.setWeight(1e-3*0 * np.eye(model.nv))
    minus.append(minu)
    ocp.stage(i).stack = pysot.AutoStack(minu)



cartesian_task = pysot.oc.SE3Task("Cartesian", ocp.stage(Ns).model, dvariables.getVariable("dq"), "base_link")
cartesian_task.setWeight(1e3 * np.eye(model.nv))

ocp.stage(Ns).stack = pysot.AutoStack(cartesian_task)

pose_ref = cartesian_task.getReference().copy()
pose_ref.translation = q_final[0:3]
pose_ref.linear = R.from_quat(q_final[3:]).as_matrix()
cartesian_task.setReference(pose_ref.copy())



ocp.update(x0, u0)
print("ocp updated!")


print("Initing solver...")
solver = pysot.swSQP(ocp)
solver.getOptions().max_iters = 1000
solver.getOptions().verbose = True
solver.getOptions().line_search_strategy = 1
solver.getOptions().beta = 1e-2
solver.getOptions().min_abs_delta_solution = 1e-6
solver.init()
print(f"{solver.getOptions().print()}")

print("...solver inited!")

space = pysot.oc.SE3Space()

rclpy.spin_once(rosnode, timeout_sec=0.0)


solver.solve(x0, u0)
x0 = solver.getStateSolution()
u0 = solver.getControlSolution()

try:
    t= 0.
    while rclpy.ok():
        input()

        x = x0[0]
        for i in range(len(x0)):
            x = x0[i]
            # x = space.integrate(x, u0[i]*dt)
            q_val = x.tolist()
            rosnode.publish(q_val)
            time.sleep(dt)

        rosnode.publish(q_val)

        rclpy.spin_once(rosnode, timeout_sec=0.0)

        time.sleep(0.001)
        
        t+= dt

except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")
    # rviz.kill()
    rosnode.destroy_node()

if rclpy.ok():
    rclpy.shutdown()

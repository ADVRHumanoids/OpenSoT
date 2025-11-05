from pyopensot.oc import *
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from xbot2_interface import pyxbot2_interface as xbi
# import pyopensot as pysot
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
from tf2_ros import TransformBroadcaster
from pyopensot.tasks.velocity import Cartesian
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from pyopensot import AffineHelper, OptvarHelper, GenericTask, Task, AffineTask, AffineConstraint

from utils import *

rviz_file_path = "/home/forest_ws/code/OpenSoT/bindings/python/examples/ocp_examples/floating_frame/floating_frame.rviz"
rviz = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '-d', f'{rviz_file_path}'], stdout=subprocess.PIPE, shell=False)

rclpy.init()
rosnode = floating_frame_node()

model = xbi.ModelInterface2(rosnode.urdf)

Ns = 40 # number of nodes
tf = 1.0 # final time
dt = tf/Ns


q_val = np.array([0., 0., 0., 0., 0., 0., 1.])
v_val = np.array([0., 0., 0., 0., 0., 0.])
a_val = np.zeros(model.nv)

q_final = np.array([-1., 0., 0., 0., 0., 0., 1.])

q_val = random_pose(-2,2)
q_final = random_pose(-2., 2.)


quats = quaternion_trajectory_numpy(Ns+1, [0,-1,0])
x0 = list()
for i in range(Ns+1):
    q0 = np.concatenate((q_val[:3] + (q_final[:3]-q_val[:3])*(i/(Ns+1)) , quats[i]))
    # x0.append(np.concatenate((q0, v_val)))
    x0.append(np.concatenate((q_val, v_val)))

u0 = list()
for i in range(Ns):
    u0.append(a_val)






vars = list()
vars.append(("q", model.nq))
vars.append(("qdot", model.nv))
vars.append(("qddot", model.nv))


variables = OptvarHelper(vars)
q = variables.getVariable("q")
qdot = variables.getVariable("qdot")
qddot = variables.getVariable("qddot")

print(f"variables.getSize(): {variables.getSize()}")



dvars = list()
dvars.append(("dq", model.nv))
dvars.append(("dqdot", model.nv))
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


ocp = OCP()
dd = list()
for i in range(Ns):
    stage = Stage()

    stage.model = xbi.ModelInterface2(rosnode.urdf)
    stage.state_space = CompositeSpace([SE3Space(), VectorSpace(6)])

    stage.x = x
    stage.dx = dx

    stage.u = qddot
    stage.du = dqddot

    stage.q = q
    stage.v = qdot

    ocp.addStage(stage)



stage = Stage()
stage.model = xbi.ModelInterface2(rosnode.urdf)
stage.state_space = CompositeSpace([pysot.oc.SE3Space(), VectorSpace(6)])
stage.x = x
stage.dx = dx
stage.q = q
stage.v = qdot
ocp.addStage(stage)

ocp.update(x0, u0)
print(f"ocp.getNumberOfNodes(): {ocp.getNumberOfNodes()}")

for i in range(Ns):
    dSE3 = pysot.oc.EulerSE3(ocp.stage(i).model, dq, dqdot, ocp.stage(i).q, ocp.stage(i).v, ocp.stage(i+1).q, dt)
    dvel = pysot.oc.EulerVector(stage.model, dqdot, dqddot, ocp.stage(i).v, ocp.stage(i).u, ocp.stage(i+1).v, dt)
    dd.append(dSE3)
    dd.append(dvel)
    ocp.stage(i).dynamics_derivative = dSE3 + dvel


ocp.update(x0, u0)

minus = list()
for i in range(Ns):
    minu = min_var.create(f"minu{i}",ocp.stage(i).u, ocp.stage(i).du)
    minu.setWeight(1e-3*0 * np.eye(model.nv))
    minus.append(minu)
    ocp.stage(i).stack = pysot.AutoStack(minu) # TODO - check why it fails withoutit


minvel = min_var.create(f"minvel", ocp.stage(Ns).v, dvariables.getVariable("dqdot"))
minvel.setWeight(1e3 * np.eye(model.nv))

cartesian_task = pysot.oc.SE3Task("Cartesian", ocp.stage(Ns).model, dvariables.getVariable("dq"), "base_link")
cartesian_task.setWeight(1e3 * np.eye(6))
ocp.stage(Ns).stack = pysot.AutoStack(cartesian_task + minvel)


ocp.update(x0, u0)
print("ocp updated!")



print("Initing solver...")
solver = pysot.swSQP(ocp)
solver.getOptions().max_iters = 1000
solver.getOptions().verbose = True
solver.getOptions().line_search_strategy = 1
solver.getOptions().beta = 1e-2
solver.init()
print(f"{solver.getOptions().print()}")
#solver.getOptions().min_abs_delta_solution = 1e-12
print("...solver inited!")


pose_ref = cartesian_task.getReference().copy()
pose_ref.translation = q_final[0:3]
pose_ref.linear = R.from_quat(q_final[3:]).as_matrix()
cartesian_task.setReference(pose_ref.copy())


solver.solve(x0, u0)
x0 = solver.getStateSolution()
u0 = solver.getControlSolution()


rosnode.publish_start(q_val)
rosnode.publish_goal(q_final)
rclpy.spin_once(rosnode, timeout_sec=0.0)


dt_sim = 0.01
try:
    t= 0.
    while rclpy.ok():


        input()

        x = x0[0]
        for i in range(len(x0)):
            x = x0[i]
            rosnode.publish(x[:7])
            time.sleep(dt)
        
        t+= dt_sim
        rclpy.spin_once(rosnode, timeout_sec=dt_sim)

except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")
    rosnode.destroy_node()

if rclpy.ok():
    rclpy.shutdown()

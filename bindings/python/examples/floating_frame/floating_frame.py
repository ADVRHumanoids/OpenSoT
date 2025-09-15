from pyopensot_oc import *
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
from tf2_ros import TransformBroadcaster
from pyopensot.tasks.velocity import Cartesian
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from pyopensot import AffineHelper, OptvarHelper, GenericTask, Task, AffineTask, AffineConstraint

class ros2_node(Node):
    def __init__(self):
        super().__init__('floating_frame')

        # Load URDF file into a string
        with open("/home/forest_ws/code/OpenSoT/bindings/python/examples/floating_frame/floating_frame.urdf", "r") as f: # TODO: Change the absolute path
            urdf_string = f.read()

        self.urdf = urdf_string
        self.declare_parameter("robot_description", urdf_string)

        # QoS: transient_local makes it latched
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # Publisher for RViz
        self.pub = self.create_publisher(String, 'robot_description', qos)

        # Publish once (or repeatedly if you want)
        msg = String()
        msg.data = self.get_parameter("robot_description").value
        self.pub.publish(msg)
        self.get_logger().info("Published robot_description on topic for RViz")
    
        self.base_link_broadcaster = TransformBroadcaster(self)

        self.w_T_b = TransformStamped()
        self.w_T_b.header.frame_id = "world"
        self.w_T_b.child_frame_id = "base_link"



    def publish(self, q_):
        q_val = q_

        self.w_T_b.header.stamp = self.get_clock().now().to_msg()
        self.w_T_b.transform.translation.x = q_val[0]
        self.w_T_b.transform.translation.y = q_val[1]
        self.w_T_b.transform.translation.z = q_val[2]
        self.w_T_b.transform.rotation.x = q_val[3]
        self.w_T_b.transform.rotation.y = q_val[4]
        self.w_T_b.transform.rotation.z = q_val[5]
        self.w_T_b.transform.rotation.w = q_val[6]


        self.base_link_broadcaster.sendTransform(self.w_T_b)


rviz_file_path = "/home/forest_ws/code/OpenSoT/bindings/python/examples/floating_frame/floating_frame.rviz"
rviz = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '-d', f'{rviz_file_path}'], stdout=subprocess.PIPE, shell=False)

rclpy.init()
rosnode = ros2_node()


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


model = xbi.ModelInterface2(rosnode.urdf)

print(f"model.nq: {model.nq}")
print(f"model.nv: {model.nv}")


q_val = np.array([0., 0., 0., 0., 0., 0., 1.])
v_val = np.array([0., 0., 0., 0., 0., 0.])

model.setJointPosition(q_val)
model.update()
T = model.getPose("base_link")
print(f"T: \n{T}")



vars = list()
# x
vars.append(("q", model.nq))
# u
vars.append(("qdot", model.nv))

variables = OptvarHelper(vars)
q = variables.getVariable("q")
qdot = variables.getVariable("qdot")


print(f"variables.getSize(): {variables.getSize()}")



dvars = list()
# dx
dvars.append(("dq", model.nv))
# du
dvars.append(("dqdot", model.nv))

dvariables = OptvarHelper(dvars)
dq = dvariables.getVariable("dq")
dqdot = dvariables.getVariable("dqdot")

print(f"dvariables.getSize(): {dvariables.getSize()}")


Ns = 20 # number of nodes
tf = 3.0 # final time
dt = tf/Ns

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

print(f"x0[0]: {x0[0]}")

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


ocp = OCP()
dd = list()
for i in range(Ns):
    stage = Stage()

    stage.model = xbi.ModelInterface2(rosnode.urdf)
    #stage.state_space = CompositeSpace([RobotSpace(stage.model)])
    stage.state_space = CompositeSpace([R3(stage.model, base="world", distal="base_link"), QuaternionSpace()])

    stage.x = x
    stage.dx = dx

    stage.u = qdot
    stage.du = dqdot

    stage.q = q
    stage.v = qdot

    df = dynamics_derivative.create(f"df{i}", euler(dx, dxdot, dt))
    dd.append(df)
    stage.dynamics_derivative = df

    ocp.addStage(stage)



stage = Stage()
stage.model = xbi.ModelInterface2(rosnode.urdf)
#stage.state_space = CompositeSpace([RobotSpace(stage.model)])
stage.state_space = CompositeSpace([R3(stage.model, base="world", distal="base_link"), QuaternionSpace()])
stage.x = x
stage.dx = dx
stage.q = q
stage.v = qdot
ocp.addStage(stage)


ocp.update(x0, u0)

print(f"ocp.getNumberOfNodes(): {ocp.getNumberOfNodes()}")



minus = list()
for i in range(Ns):
    minu = min_var.create(f"minu{i}",ocp.stage(i).u, ocp.stage(i).du)
    #minu.setWeight(1e0 * np.eye(model.nv))
    minus.append(minu)
    ocp.stage(i).stack = pysot.AutoStack(minu)



cartesian_task = Cartesian("Cartesian", ocp.stage(Ns).model, "base_link", "world")
cartesian_task.setLambda(1)
cartesian_task.setWeight(1e6 * np.eye(6))

ocp.stage(Ns).stack = pysot.AutoStack(AffineTask.toAffine(cartesian_task, dvariables.getVariable("dq")))

T, _ = cartesian_task.getReference()


ocp.update(x0, u0)
print("ocp updated!")

print(f"ocp.stage(Ns).stack.getStack()[0].getb(): {ocp.stage(Ns).stack.getStack()[0].getb()}")


print("Initing solver...")
solver = swSQP(ocp)
solver.getOptions().max_iters = 1000
solver.getOptions().verbose = True
solver.getOptions().use_line_search = True
solver.getOptions().beta = 1e-2
print(f"{solver.getOptions().print()}")
#solver.getOptions().min_abs_delta_solution = 1e-12
print("...solver inited!")

pose_ref = T.copy()
dt_sim = 0.05

space = CompositeSpace([R3(model, base="world", distal="base_link"), QuaternionSpace()])

pose_ref.translation[0] = 1.
pose_ref.translation[1] = -1.
pose_ref.translation[2] = 1.
# quat = [0., 0., 0., 0]
quat = [0.208514, 0.486534, 0.486534, 0.695048]

pose_ref.linear = R.from_quat(quat).as_matrix()

print(pose_ref)
cartesian_task.setReference(pose_ref.copy())


solver.solve(x0, u0)
x0 = solver.getStateSolution()
u0 = solver.getControlSolution()




try:
    t= 0.
    while rclpy.ok():
        rclpy.spin_once(rosnode, timeout_sec=0.0)

        input()

        x = x0[0]
        for i in range(len(x0)-1):
            # x = x0[i]
            x = space.integrate(x, u0[i]*dt)
            q_val = x.tolist()
            rosnode.publish(q_val)
            time.sleep(dt_sim)


    
        rosnode.publish(q_val)

        time.sleep(dt)
        
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

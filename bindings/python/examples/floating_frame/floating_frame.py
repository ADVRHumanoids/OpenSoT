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
from pyopensot.tasks.velocity import Cartesian
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from pyopensot import AffineHelper, OptvarHelper, GenericTask, Task, AffineTask, AffineConstraint
import random
import math

def random_quaternion():
    """
    Generate a random unit quaternion uniformly distributed on SO(3).
    """
    u1 = random.random()  # in [0,1)
    u2 = random.random()
    u3 = random.random()

    qx = math.sqrt(1 - u1) * math.sin(2 * math.pi * u2)
    qy = math.sqrt(1 - u1) * math.cos(2 * math.pi * u2)
    qz = math.sqrt(u1) * math.sin(2 * math.pi * u3)
    qw = math.sqrt(u1) * math.cos(2 * math.pi * u3)

    return np.array([qx, qy, qz, qw])

def random_pose(min, max):
    p = np.array([random.uniform(min, max), random.uniform(min, max), random.uniform(min, max)])
    r = random_quaternion()
    return np.concatenate((p, r))

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

        self.goal_broadcaster = StaticTransformBroadcaster(self)
        self.start_broadcaster = StaticTransformBroadcaster(self)

        self.w_T_b = TransformStamped()
        self.w_T_b.header.frame_id = "world"
        self.w_T_b.child_frame_id = "base_link"

        self.w_T_goal = TransformStamped()
        self.w_T_goal.header.frame_id = "world"
        self.w_T_goal.child_frame_id = "goal"

        self.w_T_start = TransformStamped()
        self.w_T_start.header.frame_id = "world"
        self.w_T_start.child_frame_id = "start"

    def publish_start(self, q):
        self.w_T_start.header.stamp = self.get_clock().now().to_msg()
        self.w_T_start.transform.translation.x = q[0]
        self.w_T_start.transform.translation.y = q[1]
        self.w_T_start.transform.translation.z = q[2]
        self.w_T_start.transform.rotation.x = q[3]
        self.w_T_start.transform.rotation.y = q[4]
        self.w_T_start.transform.rotation.z = q[5]
        self.w_T_start.transform.rotation.w = q[6]

        self.start_broadcaster.sendTransform(self.w_T_start)

    def publish_goal(self, T):
        self.w_T_goal.header.stamp = self.get_clock().now().to_msg()
        self.w_T_goal.transform.translation.x = T.translation[0]
        self.w_T_goal.transform.translation.y = T.translation[1]
        self.w_T_goal.transform.translation.z = T.translation[2]
        rot = R.from_matrix(T.linear)
        quat = rot.as_quat()
        self.w_T_goal.transform.rotation.x = quat[0]
        self.w_T_goal.transform.rotation.y = quat[1]
        self.w_T_goal.transform.rotation.z = quat[2]
        self.w_T_goal.transform.rotation.w = quat[3]

        self.goal_broadcaster.sendTransform(self.w_T_goal)
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

q_val = np.array([0., 0., 0., 0., 0., 1., 0.])#random_pose(-2., 2.)
v_val = np.array([0., 0., 0., 0., 0., 0.])

rosnode.publish_start(q_val)

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

    df = pysot.oc.SE3Derivatives(stage.model, dq, dqdot, dt)
    dd.append(df)
    stage.dynamics_derivative = df

    ocp.addStage(stage)



stage = pysot.oc.Stage()
stage.model = xbi.ModelInterface2(rosnode.urdf)
stage.state_space = pysot.oc.SE3Space()
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
    minu.setWeight(1e-9 * np.eye(model.nv))
    minus.append(minu)
    ocp.stage(i).stack = pysot.AutoStack(minu)



cartesian_task = Cartesian("Cartesian", ocp.stage(Ns).model, "base_link", "world")
cartesian_task.setLambda(1)
cartesian_task.setWeight(1e0 * np.eye(6))

ocp.stage(Ns).stack = pysot.AutoStack(AffineTask.toAffine(cartesian_task, dvariables.getVariable("dq")))

T, _ = cartesian_task.getReference()


ocp.update(x0, u0)
print("ocp updated!")

print(f"ocp.stage(Ns).stack.getStack()[0].getb(): {ocp.stage(Ns).stack.getStack()[0].getb()}")


print("Initing solver...")
solver = pysot.oc.swSQP(ocp)
solver.getOptions().max_iters = 1000
solver.getOptions().verbose = True
solver.getOptions().use_line_search = False
solver.getOptions().beta = 1e-2
print(f"{solver.getOptions().print()}")
solver.getOptions().min_abs_delta_solution = 1e-3
print("...solver inited!")

pose_ref = T.copy()
dt_sim = 0.05

space = pysot.oc.SE3Space()

q_rand = np.array([0., 0., 0., 0., 0., 0., 1.]) #random_pose(-2., 2.)
pose_ref.translation = q_rand[0:3]
pose_ref.linear = R.from_quat(q_rand[3:]).as_matrix()

print(pose_ref)
rosnode.publish_goal(pose_ref)
rclpy.spin_once(rosnode, timeout_sec=0.0)

cartesian_task.setReference(pose_ref.copy())


solver.solve(x0, u0)
x0 = solver.getStateSolution()
u0 = solver.getControlSolution()

# print(x0)
# print(u0)



try:
    t= 0.
    while rclpy.ok():
        input()

        x = x0[0]
        for i in range(len(x0)):
            x = x0[i]
            # x = space.integrate(x, u0[i]*dt - v_val*dt)
            q_val = x.tolist()
            rosnode.publish(q_val)
            time.sleep(dt_sim)


    
        rosnode.publish(q_val)

        rclpy.spin_once(rosnode, timeout_sec=0.0)

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

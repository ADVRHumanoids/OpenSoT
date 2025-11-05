import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from xbot2_interface import pyxbot2_interface as xbi
import pyopensot as pysot
import numpy as np
from std_msgs.msg import String 
from sensor_msgs.msg import JointState
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

class floating_frame_node(Node):
    def __init__(self):
        super().__init__('floating_frame')

        # Load URDF file into a string
        with open("/home/forest_ws/code/OpenSoT/bindings/python/examples/ocp_examples/floating_frame/floating_frame.urdf", "r") as f: # TODO: Change the absolute path
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

    def publish_goal(self, q):
        self.w_T_goal.header.stamp = self.get_clock().now().to_msg()
        self.w_T_goal.transform.translation.x = q[0]
        self.w_T_goal.transform.translation.y = q[1]
        self.w_T_goal.transform.translation.z = q[2]
        self.w_T_goal.transform.rotation.x = q[3]
        self.w_T_goal.transform.rotation.y = q[4]
        self.w_T_goal.transform.rotation.z = q[5]
        self.w_T_goal.transform.rotation.w = q[6]

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


class double_pendulum_node(Node):
    def __init__(self):
        super().__init__('double_pendulum')


        urdf_path = "/home/forest_ws/code/OpenSoT/bindings/python/examples/ocp_examples/double_pendulum/double_pendulum.urdf"
        mesh_base_path = "/home/forest_ws/code/OpenSoT/bindings/python/examples/ocp_examples/double_pendulum"
        
        # Load URDF file into a string
        with open(urdf_path, "r") as f:
            urdf_string = f.read()

        # Replace relative paths with absolute paths
        urdf_string = urdf_string.replace('./meshes/', f'file://{mesh_base_path}/meshes/')
        
        self.urdf = urdf_string

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)


    def publish_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)


    def publish(self,model, q_):

        msg = JointState()
        msg.name = model.getJointNames()

        msg.position = q_
        msg.header.stamp = self.get_clock().now().to_msg()

        self.joint_state_publisher.publish(msg)


class min_var(Task):
    """
    min_var consider the following function: F(var) = var - ref
    The dvariable is included to carry the information related to the size of the derivative of var
    """
    def __init__(self, name, variable, dvariable):
        super().__init__(name, dvariable.getInputSize())
        self.variable = variable
        self.dvariable = dvariable
        self.ref = 0. * self.variable.getq()
        self._W = np.eye(dvariable.getOutputSize())

    def _update(self):
        self.task =  self.dvariable + (self.variable.getValue() - self.ref)
        self._A = self.task.getM()
        self._b = -self.task.getq()

    def setReference(self, ref):
        self.ref = ref

    @classmethod
    def create(cls, name, variable, dvariable):
        obj = cls(name, variable, dvariable)
        obj.update()
        return obj

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

#simple euler
def euler(x, xdot, dt):
    return x + dt * xdot

#euler with defect
def eul(dx,du, xk,uk,xk1, dt):
    return dx + du*dt + (xk.getValue()+uk.getValue()*dt - xk1.getValue())



def quaternion_trajectory_numpy(N, axis=[0, 0, 1]):
    """
    Create quaternion trajectory using pure NumPy (axis-angle to quaternion conversion).
    """
    # Normalize the axis vector
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    
    # Create angles for full rotation
    angles = np.linspace(0, 2 * np.pi, N, endpoint=False)
    
    quaternions = []
    for angle in angles:
        # Convert axis-angle to quaternion
        half_angle = angle / 2
        w = np.cos(half_angle)
        xyz = np.sin(half_angle) * axis
        
        quaternion = np.array([xyz[0], xyz[1], xyz[2], w])
        quaternions.append(quaternion)
    
    return np.array(quaternions)
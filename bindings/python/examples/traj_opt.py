from pyopensot_oc import *
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory
from xbot2_interface import pyxbot2_interface as xbi
from pyopensot import AffineHelper, OptvarHelper, GenericTask, Task, AffineTask, AffineConstraint
from pyopensot.tasks.velocity import Cartesian
from pyopensot.constraints.velocity import JointLimits
import pyopensot as pysot
import numpy as np
from sensor_msgs.msg import JointState
import subprocess
import time
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import PoseStamped, Point
from scipy.spatial.transform import Rotation as R
import unittest
import os


np.set_printoptions(linewidth=np.inf)
class ros2_node(Node):
    def __init__(self):
        super().__init__('franka_panda_trajectory')
        self.get_logger().info("franka_panda_trajectory node has been started.")
        self.client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')

        request = GetParameters.Request()
        request.names = ['robot_description']

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        self.urdf = None
        if future.result() is not None:
            values = future.result().values
            for val in values:
                self.urdf = val.string_value
        else:
            self.get_logger().error('Failed to call service')

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        self.server = InteractiveMarkerServer(self, 'six_dof_marker_server')
        self.marker_pose = PoseStamped()

    def make_6dof_marker(self, name, pose, frame_id):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = name
        int_marker.description = '6-DOF Control'
        int_marker.scale = 0.3

        int_marker.pose.position.x = pose.translation[0]
        int_marker.pose.position.y = pose.translation[1]
        int_marker.pose.position.z = pose.translation[2]

        quat_xyzw = R.from_matrix(pose.linear).as_quat() # Format: [x, y, z, w]
        int_marker.pose.orientation.x = quat_xyzw[0]
        int_marker.pose.orientation.y = quat_xyzw[1]
        int_marker.pose.orientation.z = quat_xyzw[2]
        int_marker.pose.orientation.w = quat_xyzw[3]

        self.marker_pose.pose = int_marker.pose

        # Add a visible marker (e.g., a cube)
        cube_marker = Marker()
        cube_marker.type = Marker.CUBE
        cube_marker.scale.x = 0.05
        cube_marker.scale.y = 0.05
        cube_marker.scale.z = 0.05
        cube_marker.color.r = 0.0
        cube_marker.color.g = 1.0
        cube_marker.color.b = 0.0
        cube_marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(cube_marker)
        int_marker.controls.append(control)

        # Add 6-DOF controls
        self.add_6dof_controls(int_marker)


        self.server.insert(marker=int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()
    def process_feedback(self, feedback):
        self.marker_pose.header = feedback.header
        self.marker_pose.pose = feedback.pose
    def add_6dof_controls(self, marker):
        axes = ['x', 'y', 'z']
        for axis in axes:
            # Rotation
            control = InteractiveMarkerControl()
            control.name = f'rotate_{axis}'
            control.orientation.w = 1.0
            setattr(control.orientation, axis, 1.0)
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            marker.controls.append(control)

            # Translation
            control = InteractiveMarkerControl()
            control.name = f'move_{axis}'
            control.orientation.w = 1.0
            setattr(control.orientation, axis, 1.0)
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            marker.controls.append(control)

    def publish(self, joint_state_msg):
        self.joint_state_publisher.publish(joint_state_msg)

    def publish_horizon(self, joint_state_msg):
        self.horizon_joint_state_publisher.publish(joint_state_msg)


# Check for franka_cartesio_condif package
package_path = None
try:
    package_path = get_package_share_directory('franka_cartesio_config')
    print(f"Package path: {package_path}")
except:
    print("To run this example is needed the franka_cartesio_config package that can be download here: https://github.com/EnricoMingo/franka_cartesio_config")


roslaunch = subprocess.Popen(['ros2', 'launch', 'franka_cartesio_config', 'fp3.launch'], stdout=subprocess.PIPE, shell=False)

rviz_file_path = package_path + "/rviz/panda.rviz"
rviz = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '-d', f'{rviz_file_path}'], stdout=subprocess.PIPE, shell=False)

# Initiliaze node and wait for robot_description parameter
rclpy.init()
node = ros2_node()

Ns = 20 # number of nodes
tf = 3.0 # final time
dt = tf/Ns


model = xbi.ModelInterface2(node.urdf)
q_val = np.array([0., -0.7, 0., -2.1, 0., 1.4, 0.])
qdot_val = np.array([0., 0., 0., 0., 0., 0., 0.])
qddot_val = np.array([0., 0., 0., 0., 0., 0., 0.])

model.setJointPosition(q_val)
model.update()
T = model.getPose("fp3_link8")

"""
This set of variables describe the state and control inputs for the (NLP) OCP.
"""
vars = list()
# x
vars.append(("q", model.nq))
vars.append(("qdot", model.nv))
# u
vars.append(("qddot", model.nv))

variables = OptvarHelper(vars)
q = variables.getVariable("q")
qdot = variables.getVariable("qdot")
qddot = variables.getVariable("qddot")

print(f"variables.getSize(): {variables.getSize()}")


"""
This set of variables describe the state and control inputs for the internal liearized QP which acts in the tangent space.
In this particular case both sets of variables have the same size, but in general they could be different.
"""
dvars = list()
# dx
dvars.append(("dq", model.nv))
dvars.append(("dqdot", model.nv))
# du
dvars.append(("dqddot", model.nv))

dvariables = OptvarHelper(dvars)
dq = dvariables.getVariable("dq")
dqdot = dvariables.getVariable("dqdot")
dqddot = dvariables.getVariable("dqddot")

print(f"dvariables.getSize(): {dvariables.getSize()}")



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


x = AffineHelper.pile(q, qdot)
xdot = AffineHelper.pile(qdot, qddot)

dx = AffineHelper.pile(dq, dqdot)
dxdot = AffineHelper.pile(dqdot, dqddot)

x0 = list()
for i in range(Ns+1):
    x0.append(np.concatenate((q_val, qdot_val)))

u0 = list()
for i in range(Ns):
    u0.append(qddot_val)

print(f"x0[0]: {x0[0]}")

ocp = OCP()
dd = list()
for i in range(Ns):
    stage = Stage()
    """ First we include information related to the state space """
    stage.state_space = CompositeSpace([VectorSpace(model.nq), VectorSpace(model.nv)])

    """ We include both state variables and dvariables """
    stage.x = x
    stage.dx = dx

    """ We include both control variables and dvariables """
    stage.u = qddot
    stage.du = dqddot

    """ We include q and qdot defined for the state variables """
    stage.q = q
    stage.v = qdot

    stage.model = xbi.ModelInterface2(node.urdf)

    """ Dynamics derivative are just defined in the dvariables """
    df = dynamics_derivative.create(f"df{i}", euler(dx, dxdot, dt))
    dd.append(df)
    stage.dynamics_derivative = df

    ocp.addStage(stage)

""" Last stage (Ns) does not have dynamics and control variables/dvariables """
stage = Stage()
stage.model = xbi.ModelInterface2(node.urdf)
stage.x = x
stage.dx = dx
stage.state_space = CompositeSpace([VectorSpace(model.nq), VectorSpace(model.nq)])
stage.q = q
stage.v = qdot
ocp.addStage(stage)


ocp.update(x0, u0)

print(f"ocp.getNumberOfNodes(): {ocp.getNumberOfNodes()}")
utest = unittest.TestCase()
utest.assertTrue(ocp.getNumberOfNodes() == Ns)

minus = list()
for i in range(Ns):
    minu = min_var.create(f"minu{i}", ocp.stage(i).u, ocp.stage(i).du)
    minu.setWeight(1e0 * np.eye(model.nv))
    minus.append(minu)
    ocp.stage(i).stack = pysot.AutoStack(minu)



#
# set goal at final state
cartesian_task = Cartesian("Cartesian", ocp.stage(Ns).model, "fp3_link8", "world")
cartesian_task.setLambda(1)
cartesian_task.setWeight(1e6 * np.eye(6))

""" 
Important:  

The cartesian task define the function F(q) = f(q) - ref
and its derivative:

df(q)/dq = J(q)dq

hence the Affine task is applied to dq: [J(q) 0] [dq dqdot]' = J(q)dq

"""
ocp.stage(Ns).stack = pysot.AutoStack(AffineTask.toAffine(cartesian_task, dvariables.getVariable("dq")))

T, _ = cartesian_task.getReference()
node.make_6dof_marker(name="fp3_link8", pose=T, frame_id="world")

#
ocp.update(x0, u0)
#
print("ocp updated!")

print(f"ocp.stage(Ns).stack.getStack()[0].getb(): {ocp.stage(Ns).stack.getStack()[0].getb()}")
#




print("Initing solver...")
solver = swSQP(ocp)
solver.getOptions().max_iters = 1000
solver.getOptions().verbose = True
solver.getOptions().use_line_search = True
solver.getOptions().beta = 1e-2
print(f"{solver.getOptions().print()}")
#solver.getOptions().min_abs_delta_solution = 1e-12
print("...solver inited!")



# Set a homing configuration
#
#model.setJointPosition(q)
#model.update()

msg = JointState()
msg.name = model.getJointNames()

pose_ref = T.copy()
dt_sim = 0.05
t = 2.

last_pose_reference = pose_ref.copy()
msg.position = x0[0][:model.nq].tolist()
try:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)

        pose_ref.translation[0] = node.marker_pose.pose.position.x
        pose_ref.translation[1] = node.marker_pose.pose.position.y
        pose_ref.translation[2] = node.marker_pose.pose.position.z
        quat = [node.marker_pose.pose.orientation.x, node.marker_pose.pose.orientation.y,
                node.marker_pose.pose.orientation.z, node.marker_pose.pose.orientation.w]
        pose_ref.linear = R.from_quat(quat).as_matrix()

        if pose_ref != last_pose_reference:
            cartesian_task.setReference(pose_ref)
            last_pose_reference = pose_ref.copy()
            success = solver.solve(x0, u0)
            if(success):
                x0 = solver.getStateSolution()
                u0 = solver.getControlSolution()

                for x in x0:
                    msg.position = x[:model.nq].tolist()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    node.publish(msg)
                    time.sleep(dt_sim)

                for i in range(len(x0)):
                    x0[i] = x0[-1]
                for i in range(len(u0)):
                    u0[i] = u0[-1]

            else:
                print("problem NOT solved!")
        else:
            msg.header.stamp = node.get_clock().now().to_msg()
            node.publish(msg)



        time.sleep(dt_sim)

except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")
    roslaunch.kill()
    rviz.kill()
    node.destroy_node()

if rclpy.ok():
    rclpy.shutdown()
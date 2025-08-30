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
from ttictoc import tic, toc

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

        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

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

class min_var(Task):
    def __init__(self, name, variable):
        super().__init__(name, variable.getInputSize())
        self.variable = variable
        self._W = np.eye(variable.getOutputSize())

    def _update(self):
        self.lin =  self.variable + self.variable.getValue()
        self._A = self.lin.getM()
        self._b = -self.lin.getq()


    @classmethod
    def create(cls, name, variable):
        obj = cls(name, variable)
        obj.update()
        return obj

class dynamics_derivative(Task):
    def __init__(self, name, f):
        super().__init__(name, f.getInputSize())
        self.f = f
        self._W = np.eye(f.getOutputSize())

    def _update(self):
        self.lin = self.f
        self._A = self.lin.getM()
        self._b = -self.lin.getq()

    @classmethod
    def create(cls, name, f):
        obj = cls(name, f)
        obj.update()
        return obj

def euler(x, xdot, dt):
    return x + dt * xdot  # x1 = x0 + dt * xdot0


x = AffineHelper.pile(q, qdot)
xdot = AffineHelper.pile(qdot, qddot)

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

    stage.x = x
    stage.u = qddot
    stage.q = q
    stage.v = qdot

    stage.model = xbi.ModelInterface2(node.urdf)

    f = euler(x, xdot, dt)
    w0 = np.concatenate((x0[i], u0[i]))
    f.getValue(w0)
    stage.variables.append(f)
    df = dynamics_derivative.create(f"df{i}", f)
    dd.append(df)
    stage.dynamics_derivative = df

    ocp.addStage(stage)


stage = Stage()
stage.model = xbi.ModelInterface2(node.urdf)
stage.x = x
stage.q = q
stage.v = qdot
ocp.addStage(stage)



ocp.update(x0, u0)

print(f"ocp.getNumberOfNodes(): {ocp.getNumberOfNodes()}")
utest = unittest.TestCase()
utest.assertTrue(ocp.getNumberOfNodes() == Ns)

minus = list()
for i in range(Ns):
    minu = min_var.create(f"minu{i}", ocp.stage(i).u)
    minu.setWeight(1e0 * np.eye(model.nv))
    minus.append(minu)
    ocp.stage(i).stack = pysot.AutoStack(minu)
#
# set goal at final state
cartesian_task = Cartesian("Cartesian", ocp.stage(Ns).model, "fp3_link8", "world")
cartesian_task.setLambda(1)
cartesian_task.setWeight(1e4 * np.eye(6))

ocp.stage(Ns).stack = pysot.AutoStack(AffineTask.toAffine(cartesian_task, variables.getVariable("qdot")))

T, _ = cartesian_task.getReference()
Tr = T.copy()
Tr.translation[0] += 0.1
Tr.translation[1] += 0.1
Tr.translation[2] -= 0.1
cartesian_task.setReference(Tr)


#
ocp.update(x0, u0)
#
print("ocp updated!")

print(f"ocp.stage(Ns).stack.getStack()[0].getb(): {ocp.stage(Ns).stack.getStack()[0].getb()}")
#




print("Initing solver...")
solver = swSQP(ocp)
solver.getOptions().max_iters = 100
print("...solver inited!")

tic()
success = solver.solve(x0, u0)
elapsed = toc()  # End timer and print elapsed time
print(f"Elapsed time: {elapsed:.3f} seconds")
if(success):
    print("OCP solved!")
else:
    print("OCP not solved!")


x_solution = solver.getStateSolution()
u_solution = solver.getControlSolution()


# Publish the trajectory

try:
    while rclpy.ok():
        for i in range(Ns):
            js = JointState()
            js.header.stamp = node.get_clock().now().to_msg()
            js.name = model.getJointNames()
            js.position = x_solution[i][:model.nq].tolist()
            node.publish(js)
            time.sleep(0.1)
        time.sleep(2.)






# Set a homing configuration
#
#model.setJointPosition(q)
#model.update()




#try:
#    while rclpy.ok():
#        pass
#
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
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
model = xbi.ModelInterface2(node.urdf)
# Set a homing configuration
q = [0., -0.7, 0., -2.1, 0., 1.4, 0.]
model.setJointPosition(q)
model.update()

dt = 1./100.

nx = 3 # [px, py, pz]
nu = 3 # [vx, vy, vz]

Ns = 20 # number of nodes

tf = Ns*dt # final time

vars = list()
vars.append((f"x0", nx))
vars.append((f"qdot0", model.getNv()))
for i in range(Ns):
    vars.append((f"u{i}", nu))

variables = OptvarHelper(vars)
print(f"variables.getSize(): {variables.getSize()}")

Te = model.getPose("fp3_link8")
print(f"Te: {Te.translation}")
node.make_6dof_marker(name="fp3_link8", pose=Te, frame_id="world")

xinit = Te.translation

print(f"xinit: {xinit}")

class state(AffineHelper):
    def __init__(self, ns, variables, dt):
        super().__init__(nx, variables.getSize())
        self.ns = ns
        self.variables = variables
        self.dt = dt
        self.x = None
        self.xdot = list()

        self.init()

    def init(self):
        self.x = self.variables.getVariable("x0")
        if self.ns == 0:
            self._M = self.x.getM()
            self._q = self.x.getq()
        else:
            for i in range(self.ns):
                self.x = self.x + dt * self.variables.getVariable(f"u{i}")

            self._M = self.x.getM()
            self._q = self.x.getq()

        def update(self):
            pass


x = list()
goal_task_list = list()
initial_state = None
for i in range(Ns+1):
    x.append(state(i, variables, dt))
    x[i] = x[i] - xinit
    if i == 0:
        initial_state = GenericTask(f"goal_task_{i}", x[i])
    else:
        goal_task_i = GenericTask(f"goal_task_{i}", x[i])
        goal_task_list.append(goal_task_i)

goal_state = pysot.AggregatedTask(goal_task_list, variables.getSize())

class CartesianVelocity(AffineHelper):
    def __init__(self, frame, model, qdot):
        super().__init__(model.getNv(), 6)
        self.model = model
        self.qdot = qdot
        self.frame = frame
        self.update()

    def update(self):
        self.v = self.model.getJacobian(self.frame) @ self.qdot
        self.setM(self.v.getM())
        self.setq(self.v.getq())

vee = CartesianVelocity("fp3_link8", model, variables.getVariable("qdot0"))
vee.update()

class ik_task(Task):
    def __init__(self, vee, v):
        super().__init__("ik_task", vee.getInputSize())
        self.vee = vee
        self.v = v
        self.update()

    def _update(self):
        constr = self.v - self.vee
        self._A = constr.getM()
        self._b = -constr.getq()

    @classmethod
    def create(cls, vee, v):
        obj = cls(vee, v)
        obj.update()
        return obj

ik_task = ik_task.create(vee[0:3], variables.getVariable("u0"))

min_u_list = list()
for i in range(Ns):
    u_i = variables.getVariable(f"u{i}")
    min_ui = GenericTask(f"min_u{i}", u_i)
    min_ui.setWeight(1e-3 * np.eye(u_i.getM().shape[0]))
    min_ui.update()
    min_u_list.append(min_ui)

min_u = pysot.AggregatedTask(min_u_list, variables.getSize())

c = Cartesian("Cartesian", model, "fp3_link8", "world")
c.setLambda(10.)

qmin, qmax = model.getJointLimits()
qlims = JointLimits(model, qmax, qmin)
qlims.setBoundScaling(1./dt)


stack = pysot.AutoStack(ik_task + min_u + goal_task_list[Ns-1] + AffineTask.toAffine(c[3:], variables.getVariable("qdot0"))) << initial_state << AffineConstraint.toAffine(qlims, variables.getVariable("qdot0"))
stack.update()
solver = pysot.iHQP(stack)

msg = JointState()
msg.name = model.getJointNames()

x0 = xinit
pose_ref = Te.copy()
try:
    while rclpy.ok():
        model.setJointPosition(q)
        model.update()

        for xi in x:
            xi.update()
        vee.update()

        #for i in range(1, Ns):
        #    goal_task_list[i-1].setb(goal_task_list[i].getb())

        pose_ref.translation[0] = node.marker_pose.pose.position.x
        pose_ref.translation[1] = node.marker_pose.pose.position.y
        pose_ref.translation[2] = node.marker_pose.pose.position.z
        quat = [node.marker_pose.pose.orientation.x, node.marker_pose.pose.orientation.y,
                node.marker_pose.pose.orientation.z, node.marker_pose.pose.orientation.w]
        pose_ref.linear = R.from_quat(quat).as_matrix()
        goal_task_list[Ns-1].setb(pose_ref.translation)

        c.setReference(pose_ref)


        x0 = model.getPose("fp3_link8").translation
        initial_state.setb(x0)

        stack.update()

        w = solver.solve()

        qdot0 = variables.getVariable("qdot0").getValue(w)

        q += dt * qdot0

        # Publish joint states
        msg.position = q
        msg.header.stamp = node.get_clock().now().to_msg()

        rclpy.spin_once(node, timeout_sec=0.0)
        node.publish(msg)

        time.sleep(dt)
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
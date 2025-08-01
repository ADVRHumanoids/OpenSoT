import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory
from xbot2_interface import pyxbot2_interface as xbi
from pyopensot.tasks.acceleration import Cartesian, CoM, DynamicFeasibility
from pyopensot.constraints.acceleration import JointLimits, VelocityLimits
from pyopensot.constraints.force import FrictionCone
from pyopensot.variables import Torque
from pyopensot.tasks import MinimizeVariable
import pyopensot as pysot
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, WrenchStamped
from tf2_ros import TransformBroadcaster
import subprocess
import time

class ros2_node(Node):
    def __init__(self):
        super().__init__('LittleDog_id')
        self.get_logger().info("LittleDog ID node has been started.")
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

        self.force_publishers = {}

        self.base_link_broadcaster = TransformBroadcaster(self)

    def initialize_force_publishers(self, contact_frames):
        for contact_frame in contact_frames:
            self.force_publishers[contact_frame] = self.create_publisher(WrenchStamped, contact_frame, 10)

    def publish(self, joint_state_msg, transform_msg, force_msgs = None):
        self.joint_state_publisher.publish(joint_state_msg)
        self.base_link_broadcaster.sendTransform(transform_msg)

        if force_msgs is not None:
            for contact_frame, force_msg in force_msgs.items():
                self.force_publishers[contact_frame].publish(force_msg)



package_path = None
try:
    package_path = get_package_share_directory('LittleDog')
    print(f"Package path: {package_path}")
except:
    print("To run this example is needed the LittleDog package ([ros2 branch]) that can be download here: https://github.com/EnricoMingo/LittleDog")

roslaunch = subprocess.Popen(['ros2', 'launch', 'LittleDog', 'LittleDog.launch'], stdout=subprocess.PIPE, shell=False)
rviz_file_path = package_path + "/launch/LittleDog.rviz"
rviz = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2',  '-d', f'{rviz_file_path}'], stdout=subprocess.PIPE, shell=False)

# Initiliaze node and wait for robot_description parameter
rclpy.init()
node = ros2_node()


model = xbi.ModelInterface2(node.urdf)
qmin, qmax = model.getJointLimits()
dqmax = model.getVelocityLimits()
q = [0., 0., 0., 0., 0., 0., 1.,
     0., -0.7, 1.4, 0., -0.7, 1.4, 0., 0.7, -1.4, 0., 0.7, -1.4]
dq = np.zeros(model.nv)
model.setJointPosition(q)
model.setJointVelocity(dq)
model.update()

dt = 1./1000.

# Instantiate Variables: qddot and contact forces (3 per contact)
contact_frames = ["front_left_foot_center", "front_right_foot_center", "back_left_foot_center", "back_right_foot_center"]
variables_vec = dict()
variables_vec["qddot"] = model.nv
for contact_frame in contact_frames:
    variables_vec[contact_frame] = 3
variables = pysot.OptvarHelper(variables_vec)

# Creates tasks cand constraints
com = CoM(model, variables.getVariable("qddot"))
com.setLambda(1.)
com_ref, vel_ref, acc_ref = com.getReference()
com0 = com_ref.copy()

base = Cartesian("base", model, "world", "body", variables.getVariable("qddot"))
base.setLambda(1.)

contact_tasks = list()
for contact_frame in contact_frames:
    contact_tasks.append(Cartesian(contact_frame + "_kin", model, contact_frame, "world", variables.getVariable("qddot")))

stack = 0.1*com + 0.1*(base%[3, 4, 5])
force_variables = list()
for i in range(len(contact_frames)):
    stack = stack + 10.*(contact_tasks[i]%[0, 1, 2])
    force_variables.append(variables.getVariable(contact_frames[i]))

torques = Torque(model=model, qddot_var=variables.getVariable("qddot"), contact_links=contact_frames, force_vars=force_variables)
stack = stack + 1e-3 * MinimizeVariable("min_torques", torques)

# Creates the stack.
# Notice:  we do not need to keep track of the DynamicFeasibility constraint so it is created when added into the stack.
# The same can be done with other constraints such as Joint Limits and Velocity Limits
stack = pysot.AutoStack(stack) << DynamicFeasibility("floating_base_dynamics", model, variables.getVariable("qddot"), force_variables, contact_frames)
stack = stack << JointLimits(model, variables.getVariable("qddot"), qmax, qmin, 10.*dqmax, dt)
stack = stack << VelocityLimits(model, variables.getVariable("qddot"), dqmax, dt)
for i in range(len(contact_frames)):
    T = model.getPose(contact_frames[i])
    mu = (T.linear, 0.8) # rotation is world to contact
    stack = stack << FrictionCone(contact_frames[i], variables.getVariable(contact_frames[i]), model, mu)

# Creates the solver
solver = pysot.iHQP(stack)

# Initialize the node
node.initialize_force_publishers(contact_frames)

msg = JointState()
msg.name = model.getJointNames()[1::]

w_T_b = TransformStamped()
w_T_b.header.frame_id = "world"
w_T_b.child_frame_id = "body"

force_msgs = {}
for contact_frame in contact_frames:
    force_msgs[contact_frame] = WrenchStamped()
    force_msgs[contact_frame].header.frame_id = contact_frame
    force_msgs[contact_frame].wrench.torque.x = force_msgs[contact_frame].wrench.torque.y = force_msgs[contact_frame].wrench.torque.z = 0.
#
t = 0.
alpha = 0.05
try:
    while rclpy.ok():
        # Update actual position in the model
        model.setJointPosition(q)
        model.setJointVelocity(dq)
        model.update()

        # Variable Update
        torques.update()
#
        # Compute new reference for CoM task
        com_ref[2] = com0[2] + alpha * np.sin(3.1415 * t)
        com_ref[1] = com0[1] + alpha * np.cos(3.1415 * t)
        t = t + dt
        com.setReference(com_ref)

        # Update Stack
        stack.update()
#
        # Solve
        x = solver.solve()
        ddq = variables.getVariable("qddot").getValue(x) # from variables vector we retrieve the joint accelerations
        q = model.sum(q, dq*dt + 0.5 * ddq * dt * dt) # we use the model sum to account for the floating-base
        dq += ddq*dt
#
        # Publish joint states
        msg.position = q[7::]
        msg.header.stamp = node.get_clock().now().to_msg()
#
        w_T_b.header.stamp = msg.header.stamp
        w_T_b.transform.translation.x = q[0]
        w_T_b.transform.translation.y = q[1]
        w_T_b.transform.translation.z = q[2]
        w_T_b.transform.rotation.x = q[3]
        w_T_b.transform.rotation.y = q[4]
        w_T_b.transform.rotation.z = q[5]
        w_T_b.transform.rotation.w = q[6]
#
        for contact_frame in contact_frames:
            T = model.getPose(contact_frame)
            force_msgs[contact_frame].header.stamp = msg.header.stamp
            f_local = T.linear.transpose() @ variables.getVariable(contact_frame).getValue(x) # here we compute the value of the contact forces in local frame from world frame
            force_msgs[contact_frame].wrench.force.x = f_local[0]
            force_msgs[contact_frame].wrench.force.y = f_local[1]
            force_msgs[contact_frame].wrench.force.z = f_local[2]
#
        rclpy.spin_once(node, timeout_sec=0.0)
        node.publish(msg, w_T_b, force_msgs=force_msgs)
#
#       # Sleep to maintain the desired rate
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
import subprocess
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from xbot2_interface import pyxbot2_interface as xbi
from xbot2_interface import pyxbot2_collision
from xbot2_interface import pyaffine3
import time
import numpy as np
import array

from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Point

import pyopensot as pysot
from pyopensot.tasks.velocity import Postural, Cartesian
from pyopensot.constraints.velocity import JointLimits, VelocityLimits
from pyopensot_collision.constraints.velocity import CollisionAvoidance

from std_srvs.srv import SetBool

class ros2_node(Node):
    def __init__(self):
        super().__init__('tiago_dual_collision_avoidance')
        self.get_logger().info("tiago_dual node has been started.")
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
        self.base_link_broadcaster = TransformBroadcaster(self)

        self.server = InteractiveMarkerServer(self, 'six_dof_marker_server')
        self.marker_pose = PoseStamped()

        self.collision_distances_publisher = self.create_publisher(Marker, 'collision_distances', 10)

        self.srv = self.create_service(SetBool, 'enable_external_obstacle', self.handle_request) # ros2 service call /enable_external_obstacle std_srvs/srv/SetBool "{data: True}"
        self.enable_external_obstacle = False
        self.world_object_publisher = self.create_publisher(Marker, 'world_object', 10)

        self.cube = Marker()
        self.cube.header.frame_id = "world"
        self.cube.ns = "environment"
        self.cube.id = 0
        self.cube.type = Marker.CUBE
        self.cube.scale.x = 0.1
        self.cube.scale.y = 0.6
        self.cube.scale.z = 1.4
        self.cube.color.g = 1.0
        self.cube.color.a = 0.5
        self.cube.pose.position.x = 0.75
        self.cube.pose.position.y = 0.0
        self.cube.pose.position.z = 0.75
        self.cube.pose.orientation.x = self.cube.pose.orientation.y = self.cube.pose.orientation.z = 0.0
        self.cube.pose.orientation.w = 1.0

    def handle_request(self, request, response):
        self.get_logger().info(f"Received request: enable_external_obstacle = {request.data}")
        self.enable_external_obstacle = request.data
        response.success = True
        response.message = f"Received {request.data}"
        return response

    def publishObstacle(self, time, action):
        if self.enable_external_obstacle:
            self.cube.header.stamp = time
            self.cube.action = action
            self.world_object_publisher.publish(self.cube)


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

    def toJointStateMsg(self, joint_state_msg, q):
        joint_state_msg.position[0] = np.arctan2(np.sin(q[8]), np.cos(q[7]))
        joint_state_msg.position[1] = np.arctan2(np.sin(q[10]), np.cos(q[9]))
        joint_state_msg.position[2] = np.arctan2(np.sin(q[12]), np.cos(q[11]))
        joint_state_msg.position[3] = np.arctan2(np.sin(q[14]), np.cos(q[13]))
        joint_state_msg.position[4:] = array.array('d', q[15:])

    def publish(self, joint_state_msg, transform_msg):
        self.joint_state_publisher.publish(joint_state_msg)
        self.base_link_broadcaster.sendTransform(transform_msg)

    def publishCollisionDistances(self, collision_distance_points, time):
        marker = Marker()
        marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0
        marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.header.frame_id = "world"
        marker.header.stamp = time
        marker.ns = "collision_distances"
        marker.id = 0
        marker.scale.x = 0.005  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Opaque

        for point_pairs in collision_distance_points:
            pa = point_pairs[0]
            pb = point_pairs[1]

            point_a = Point()
            point_a.x = pa[0]
            point_a.y = pa[1]
            point_a.z = pa[2]

            point_b = Point()
            point_b.x = pb[0]
            point_b.y = pb[1]
            point_b.z = pb[2]

            marker.points.append(point_a)
            marker.points.append(point_b)


        self.collision_distances_publisher.publish(marker)



package_path = None
try:
    package_path = get_package_share_directory('tiago_dual_cartesio_config')
    print(f"Package path: {package_path}")
except:
    print("To run this example is needed the tiago_dual_cartesio_config package that can be download here: https://github.com/hucebot/tiago_dual_cartesio_config")

launch_path = "visualizer.launch"
roslaunch = subprocess.Popen(['ros2', 'launch', launch_path], stdout=subprocess.PIPE, shell=False)
rviz = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '-d', 'tiago_dual.rviz'], stdout=subprocess.PIPE, shell=False)


rclpy.init()

node = ros2_node()
model = xbi.ModelInterface2(node.urdf)

# Set a homing configuration
q = [0., 0., 0., .0, 0., 0., 1., # floating_base
     np.cos(0.), np.sin(0.),     # 'wheel_front_left_joint'
     np.cos(0.), np.sin(0.),     # 'wheel_front_right_joint'
     np.cos(0.), np.sin(0.),     # 'wheel_rear_left_joint'
     np.cos(0.), np.sin(0.),     # 'wheel_rear_right_joint'
     0.,                # 'torso_lift_joint'
     0., 1.5, 1.54, 1.75, 0., 0., 0., # 'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint'
     0., 1.5, 1.54, 1.75, 0., 0., 0., # 'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint'
     0., 0.] # 'head_1_joint', 'head_2_joint'
model.setJointPosition(q)
model.update()

msg = JointState()
msg.name = model.getJointNames()[1::]
msg.position = [0.0] * len(msg.name)

w_T_b = TransformStamped()
w_T_b.header.frame_id = "world"
w_T_b.child_frame_id = "base_footprint"

dt = 1./100.

## CREATE OPTIMIZATION PROBLEM
manipulation_base_frame = "world"
#1. TASKS
gripper_left = Cartesian("Cartesian", model, "gripper_left_grasping_frame", manipulation_base_frame)
gripper_left.setLambda(0.1)

gripper_right = Cartesian("Cartesian", model, "gripper_right_grasping_frame", manipulation_base_frame)
gripper_right.setLambda(0.1)

base = Cartesian("Cartesian", model, "base_link", "world")
base.setLambda(0.1)

postural = Postural(model)
postural.setLambda(0.05)
Wpostural = postural.getWeight()
Wpostural[0:6] = 0.0  # Do not penalize floating base
Wpostural[6:10] = 0.0  # Do not penalize wheels
postural.setWeight(Wpostural)

# CONSTRAINTS
qmin, qmax = model.getJointLimits()
qlims = JointLimits(model, qmax, qmin)
#
dqmax = model.getVelocityLimits()
dqlims = VelocityLimits(model, dqmax, dt)

# Base2D:
base2D = Cartesian("Cartesian", model, "base_link", "world")
base2D.setLambda(0.1)

# Collision Avoidance
urdf_string = node.urdf
srdf_path = package_path + "/capsules/srdf/tiago_dual_capsules.srdf"
srfd_string = None
with open(srdf_path, 'r') as f:
    srdf_string = f.read()
collision_avoidance = CollisionAvoidance(model, max_pairs=100, collision_urdf=urdf_string, collision_srdf=srdf_string)
collision_avoidance.setBoundScaling(0.1)
collision_avoidance.setLinkPairThreshold(0.01)
collision_avoidance.setDetectionThreshold(-1)
collision_list = {
    ("arm_left_3_link", "base_link"),
    ("arm_left_5_link", "base_link"),
    ("gripper_left_left_finger_link", "base_link"),
    ("gripper_left_right_finger_link", "base_link"),
    ("gripper_left_link", "base_link"),

    ("arm_left_3_link", "head_2_link"),
    ("arm_left_5_link", "head_2_link"),
    ("gripper_left_left_finger_link", "head_2_link"),
    ("gripper_left_right_finger_link", "head_2_link"),
    ("gripper_left_link", "head_2_link"),

    ("arm_left_3_link", "torso_lift_link"),
    ("arm_left_5_link", "torso_lift_link"),
    ("gripper_left_left_finger_link", "torso_lift_link"),
    ("gripper_left_right_finger_link", "torso_lift_link"),
    ("gripper_left_link", "torso_lift_link"),

    ("arm_right_3_link", "base_link"),
    ("arm_right_5_link", "base_link"),
    ("gripper_right_left_finger_link", "base_link"),
    ("gripper_right_right_finger_link", "base_link"),
    ("gripper_right_link", "base_link"),

    ("arm_right_3_link", "head_2_link"),
    ("arm_right_5_link", "head_2_link"),
    ("gripper_right_left_finger_link", "head_2_link"),
    ("gripper_right_right_finger_link", "head_2_link"),
    ("gripper_right_link", "head_2_link"),

    ("arm_right_3_link", "torso_lift_link"),
    ("arm_right_5_link", "torso_lift_link"),
    ("gripper_right_left_finger_link", "torso_lift_link"),
    ("gripper_right_right_finger_link", "torso_lift_link"),
    ("gripper_right_link", "torso_lift_link"),

    ("gripper_right_left_finger_link", "gripper_left_left_finger_link"),
    ("gripper_right_left_finger_link", "gripper_left_right_finger_link"),
    ("gripper_right_right_finger_link", "gripper_left_left_finger_link"),
    ("gripper_right_right_finger_link", "gripper_left_right_finger_link"),
    ("gripper_right_left_finger_link", "gripper_left_link"),
    ("gripper_right_right_finger_link", "gripper_left_link"),
    ("gripper_left_left_finger_link", "gripper_right_link"),
    ("gripper_left_right_finger_link", "gripper_right_link"),
    ("gripper_left_link", "gripper_right_link"),

    ("gripper_left_link", "arm_right_5_link"),
    ("gripper_right_link", "arm_left_5_link"),
    ("arm_left_5_link", "arm_right_5_link"),
    ("arm_left_5_link", "arm_right_4_link"),
    ("arm_left_4_link", "arm_right_5_link"),
    ("gripper_left_link", "arm_right_4_link"),
    ("gripper_left_link", "arm_right_5_link"),
    ("gripper_right_link", "arm_left_4_link"),
    ("gripper_right_link", "arm_left_5_link"),

    ("torso_fixed_column_link", "gripper_right_left_finger_link"),
    ("torso_fixed_column_link", "gripper_right_right_finger_link"),
    ("torso_fixed_column_link", "gripper_left_left_finger_link"),
    ("torso_fixed_column_link", "gripper_left_right_finger_link")
}
collision_avoidance.setCollisionList(collision_list)



# STACK
stack = ( (gripper_left + gripper_right + base%[0, 1, 5]) / postural) << qlims << dqlims << collision_avoidance << base2D%[2, 3, 4]
stack.update()

# SOLVER
solver = pysot.iHQP(stack)


#SIMPLE CARTESIAN TRAJECTORY
pose_ref, vel_ref = gripper_right.getReference()
print(f"pose_ref: {pose_ref}")
print(f"vel_ref: {vel_ref}")
node.make_6dof_marker(name="gripper_right_marker", pose=pose_ref, frame_id=manipulation_base_frame)

object_in_scene = False
try:
    while rclpy.ok():
        # Update actual position in the model
        model.setJointPosition(q)
        model.update()

        # Update ref
        pose_ref.translation[0] = node.marker_pose.pose.position.x
        pose_ref.translation[1] = node.marker_pose.pose.position.y
        pose_ref.translation[2] = node.marker_pose.pose.position.z
        quat = [node.marker_pose.pose.orientation.x, node.marker_pose.pose.orientation.y, node.marker_pose.pose.orientation.z, node.marker_pose.pose.orientation.w]
        pose_ref.linear = R.from_quat(quat).as_matrix()

        gripper_right.setReference(pose_ref, vel_ref)

        #Environment Collision
        if node.enable_external_obstacle:
            if not collision_avoidance.setCollisionShapeActive("mybox", True):
                box = pyxbot2_collision.shape.Box()
                box.size = np.array([node.cube.scale.x, node.cube.scale.y, node.cube.scale.z])
                w_T_c = pyaffine3.Affine3()
                w_T_c.translation = np.array([node.cube.pose.position.x, node.cube.pose.position.y, node.cube.pose.position.z])
                w_T_c.linear = R.from_quat([node.cube.pose.orientation.x, node.cube.pose.orientation.y, node.cube.pose.orientation.z, node.cube.pose.orientation.w]).as_matrix()
                collision_avoidance.addCollisionShape("mybox", "world", box, w_T_c, [])
                object_in_scene = True
        elif not node.enable_external_obstacle and object_in_scene:
            collision_avoidance.setCollisionShapeActive("mybox", False)

        # Update Stack
        stack.update()


        # Solve
        dq = solver.solve()
        q = model.sum(q, dq)  # we use the model sum to account for the floating-base and wheels manifolds

        # SSend results to ROS2
        node.toJointStateMsg(msg, q)
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
        rclpy.spin_once(node, timeout_sec=0.0)
        node.publish(msg, w_T_b)

        node.publishCollisionDistances(collision_avoidance.getOrderedWitnessPointVector(), msg.header.stamp)
        if node.enable_external_obstacle:
            node.publishObstacle(msg.header.stamp, Marker.ADD)
        else:
            node.publishObstacle(msg.header.stamp, Marker.DELETE)

        #
        time.sleep(dt)

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

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
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

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

# Initiliaze node and wait for robot_description parameter
rclpy.init()
node = ros2_node()



model = xbi.ModelInterface2(node.urdf)

print(f"model.nq: {model.nq}")
print(f"model.nv: {model.nv}")


q_val = np.array([1., 1., 1., 0., 0., 0., 1.])
qdot_val = np.array([0., 0., 0., 0., 0., 1.])

# model.setJointPosition(q_val)
# model.update()
# T = model.getPose("base_link")
# print(f"T: \n{T}")

SE3 = CompositeSpace([VectorSpace(3), QuaternionSpace()])



dt = 1./1000.
try:
    t= 0.
    while rclpy.ok():
        # qdot_val[0] = -0.5 * np.sin(t)
        # qdot_val[1] = 0.5 * np.cos(t)

        q_val = SE3.integrate(q_val, qdot_val*dt)


        rclpy.spin_once(node, timeout_sec=0.0)

        node.publish(q_val)

        time.sleep(dt)
        
        t+= dt

except KeyboardInterrupt:
    print("KeyboardInterrupt: Stopping the node.")
    pass
finally:
    print("Stopping the node.")
    rviz.kill()
    node.destroy_node()

if rclpy.ok():
    rclpy.shutdown()

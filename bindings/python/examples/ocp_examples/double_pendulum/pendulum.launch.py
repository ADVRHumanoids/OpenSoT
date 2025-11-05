from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot'
    )
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    
    # URDF paths
    urdf_path = "/home/forest_ws/code/OpenSoT/bindings/python/examples/ocp_examples/double_pendulum/double_pendulum.urdf"
    mesh_base_path = "/home/forest_ws/code/OpenSoT/bindings/python/examples/ocp_examples/double_pendulum"
    
    # Load URDF file into a string
    with open(urdf_path, "r") as f:
        urdf_string = f.read()

    # Replace relative paths with absolute paths
    urdf_string = urdf_string.replace('./meshes/', f'file://{mesh_base_path}/meshes/')
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': urdf_string
        }],
        output='screen'
    )
    
    return LaunchDescription([
        namespace_arg,
        robot_state_publisher_node
    ])
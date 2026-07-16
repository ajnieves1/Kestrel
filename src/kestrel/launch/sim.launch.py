# Launch the Gazebo world, SITL with the Gazebo frame, MAVROS, and the camera bridge
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Build the launch description with Gazebo, SITL, MAVROS, and the camera bridge
def generate_launch_description():
    package_share = get_package_share_directory('kestrel')
    params_file = os.path.join(package_share, 'config', 'kestrel_params.yaml')
    world_file = os.path.join(package_share, 'worlds', 'pylon_world.sdf')

    headless = LaunchConfiguration('headless')
    headless_argument = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo without a graphical client when true')

    # Headless runs the Gazebo server only, otherwise start the client too
    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_file],
        output='screen',
        condition=IfCondition(headless))
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        condition=UnlessCondition(headless))

    # SITL drives the Gazebo iris frame over the JSON FDM interface
    sitl_process = ExecuteProcess(
        cmd=['bash', '/ws/src/kestrel/scripts/run_sitl.sh',
             '-f', 'gazebo-iris', '--model', 'JSON',
             '--mavproxy-args=--daemon'],
        output='screen')

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[params_file],
        output='screen')

    # Bridge the Gazebo camera image and info topics into ROS
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen')

    return LaunchDescription([
        headless_argument, gazebo_headless, gazebo_gui, sitl_process,
        mavros_node, camera_bridge])

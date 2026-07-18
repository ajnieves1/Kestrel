# Launch the Gazebo world, SITL with the Gazebo frame, MAVROS, and the camera bridge
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Read the site overlay for the world file, then build the sim actions
def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory('kestrel')
    params_file = os.path.join(package_share, 'config', 'kestrel_params.yaml')
    run_sitl_script = os.path.join(package_share, 'scripts', 'run_sitl.sh')

    site = LaunchConfiguration('site').perform(context)
    site_overlay_file = os.path.join(package_share, 'config', 'sites', f'{site}.yaml')
    with open(site_overlay_file) as overlay_stream:
        site_overlay = yaml.safe_load(overlay_stream)
    world_name = site_overlay['/**']['ros__parameters']['world']
    world_file = os.path.join(package_share, 'worlds', f'{world_name}.sdf')

    headless = LaunchConfiguration('headless')

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
        cmd=['bash', run_sitl_script,
             '-f', 'gazebo-iris', '--model', 'JSON',
             '--mavproxy-args=--daemon'],
        output='screen')

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[params_file, site_overlay_file],
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

    return [gazebo_headless, gazebo_gui, sitl_process, mavros_node, camera_bridge]


# Build the launch description with Gazebo, SITL, MAVROS, and the camera bridge
def generate_launch_description():
    headless_argument = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo without a graphical client when true')
    site_argument = DeclareLaunchArgument(
        'site', default_value='pylon',
        description='Site overlay under config/sites selecting the world and geometry')

    return LaunchDescription([
        headless_argument, site_argument, OpaqueFunction(function=launch_setup)])

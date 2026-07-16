# Launch ArduCopter SITL and the MAVROS bridge
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node


# Build the launch description with SITL and MAVROS
def generate_launch_description():
    # Installed params file holds the fcu url so nothing is hardcoded here
    params_file = os.path.join(
        get_package_share_directory('kestrel'), 'config', 'kestrel_params.yaml')

    # Unused here, present for interface consistency with later launch files
    headless_argument = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Accepted for consistency, this launch is always headless')

    # MAVProxy needs daemon mode, launch gives child processes no terminal
    sitl_process = ExecuteProcess(
        cmd=['bash', '/ws/src/kestrel/scripts/run_sitl.sh',
             '--mavproxy-args=--daemon'],
        output='screen')

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[params_file],
        output='screen')

    return LaunchDescription([headless_argument, sitl_process, mavros_node])

# Launch the full inspection mission: sim, flight stack, and mission director
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Build the launch description for a full autonomous inspection mission
def generate_launch_description():
    package_share = get_package_share_directory('kestrel')
    params_file = os.path.join(package_share, 'config', 'kestrel_params.yaml')

    headless_argument = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo without a graphical client when true')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'sim.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items())

    telemetry_monitor_node = Node(
        package='kestrel', executable='telemetry_monitor',
        parameters=[params_file], output='screen')
    flight_commander_node = Node(
        package='kestrel', executable='flight_commander',
        parameters=[params_file], output='screen')
    safety_guard_node = Node(
        package='kestrel', executable='safety_guard',
        parameters=[params_file], output='screen')
    defect_detector_node = Node(
        package='kestrel', executable='defect_detector',
        parameters=[params_file], output='screen')
    mission_director_node = Node(
        package='kestrel', executable='mission_director',
        parameters=[params_file], output='screen')
    # report_writer joins here in task 14

    return LaunchDescription([
        headless_argument, sim_launch, telemetry_monitor_node,
        flight_commander_node, safety_guard_node, defect_detector_node,
        mission_director_node])

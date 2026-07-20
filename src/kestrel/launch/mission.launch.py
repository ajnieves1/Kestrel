# Launch the full inspection mission: sim, flight stack, and mission director
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Read the site argument and build the sim include plus every flight stack node
def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory('kestrel')
    params_file = os.path.join(package_share, 'config', 'kestrel_params.yaml')

    site = LaunchConfiguration('site').perform(context)
    site_overlay_file = os.path.join(package_share, 'config', 'sites', f'{site}.yaml')
    node_params = [params_file, site_overlay_file]

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'site': LaunchConfiguration('site'),
        }.items())

    telemetry_monitor_node = Node(
        package='kestrel', executable='telemetry_monitor',
        parameters=node_params, output='screen')
    flight_commander_node = Node(
        package='kestrel', executable='flight_commander',
        parameters=node_params, output='screen')
    safety_guard_node = Node(
        package='kestrel', executable='safety_guard',
        parameters=node_params, output='screen')
    defect_detector_node = Node(
        package='kestrel', executable='defect_detector',
        parameters=node_params, output='screen')
    mission_director_node = Node(
        package='kestrel', executable='mission_director',
        parameters=node_params, output='screen')
    report_writer_node = Node(
        package='kestrel', executable='report_writer',
        parameters=node_params, output='screen')
    health_monitor_node = Node(
        package='kestrel', executable='health_monitor',
        parameters=node_params, output='screen')
    obstacle_monitor_node = Node(
        package='kestrel', executable='obstacle_monitor',
        parameters=node_params, output='screen')

    return [
        sim_launch, telemetry_monitor_node, flight_commander_node,
        safety_guard_node, defect_detector_node, mission_director_node,
        report_writer_node, health_monitor_node, obstacle_monitor_node]


# Build the launch description for a full autonomous inspection mission
def generate_launch_description():
    headless_argument = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo without a graphical client when true')
    site_argument = DeclareLaunchArgument(
        'site', default_value='pylon',
        description='Site overlay under config/sites selecting the world and geometry')

    return LaunchDescription([
        headless_argument, site_argument, OpaqueFunction(function=launch_setup)])

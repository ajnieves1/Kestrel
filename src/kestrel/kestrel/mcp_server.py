# Bridge the drone's guarded flight services and telemetry into MCP tools
# MCP runs over stdio and owns stdout, this file must never print() to it,
# rclpy loggers go to stderr which is safe
import math
import os
import threading
import time

import rclpy
from fastmcp import FastMCP
from geometry_msgs.msg import PoseStamped
from kestrel_msgs.srv import GotoLocal, Takeoff
from mavros_msgs.msg import State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger

mcp = FastMCP('kestrel')
bridge_node = None


# Bridge the drone's guarded flight services and telemetry into one node
class McpBridge(Node):
    # Create service clients and telemetry subscriptions
    def __init__(self):
        super().__init__('mcp_bridge')

        self.callback_group = ReentrantCallbackGroup()

        self.state_message = None
        self.pose_message = None
        self.battery_message = None
        self.mission_state_message = None

        self.create_subscription(
            State, '/mavros/state', self.on_state,
            qos_profile_sensor_data, callback_group=self.callback_group)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.on_pose,
            qos_profile_sensor_data, callback_group=self.callback_group)
        self.create_subscription(
            BatteryState, '/mavros/battery', self.on_battery,
            qos_profile_sensor_data, callback_group=self.callback_group)
        self.create_subscription(
            String, '/kestrel/mission_state', self.on_mission_state,
            10, callback_group=self.callback_group)

        self.takeoff_client = self.create_client(
            Takeoff, '/kestrel/cmd/takeoff', callback_group=self.callback_group)
        self.goto_client = self.create_client(
            GotoLocal, '/kestrel/cmd/goto', callback_group=self.callback_group)
        self.land_client = self.create_client(
            Trigger, '/kestrel/cmd/land', callback_group=self.callback_group)
        self.abort_client = self.create_client(
            Trigger, '/kestrel/abort', callback_group=self.callback_group)

    # Store the latest vehicle state message
    def on_state(self, state_message):
        self.state_message = state_message

    # Store the latest local position message
    def on_pose(self, pose_message):
        self.pose_message = pose_message

    # Store the latest battery message
    def on_battery(self, battery_message):
        self.battery_message = battery_message

    # Store the latest mission state message
    def on_mission_state(self, state_message):
        self.mission_state_message = state_message

    # Call a kestrel service and wait for the reply with a deadline
    def call_service_blocking(self, client, request, timeout_seconds):
        service_wait_deadline = time.time() + 10.0
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() > service_wait_deadline:
                return 'service unavailable, is the flight stack running'

        future = client.call_async(request)

        deadline = time.time() + timeout_seconds
        while time.time() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.05)

        return 'the request timed out waiting for a reply'


# Arm the copter and climb to the given altitude
@mcp.tool
def takeoff(altitude: float) -> str:
    """Arm the copter and climb to the given altitude in meters."""
    if bridge_node is None:
        return 'bridge not started, launch the flight stack first'
    if not (1.0 <= altitude <= 30.0):
        return 'altitude must be between 1 and 30 meters'

    request = Takeoff.Request()
    request.altitude = altitude
    result = bridge_node.call_service_blocking(bridge_node.takeoff_client, request, 180.0)
    if isinstance(result, str):
        return result
    if result.success:
        return f'takeoff succeeded: {result.message}'
    return f'takeoff failed: {result.message}'


# Fly to a local position in meters north and east of home
@mcp.tool
def goto(north: float, east: float, altitude: float, yaw_deg: float = 0.0) -> str:
    """Fly to a local position in meters north and east of home at the given altitude, yaw_deg is compass heading."""
    if bridge_node is None:
        return 'bridge not started, launch the flight stack first'
    if math.sqrt(north ** 2 + east ** 2) > 95.0:
        return 'target is outside the 95 meter operating area'
    if not (1.0 <= altitude <= 30.0):
        return 'altitude must be between 1 and 30 meters'

    request = GotoLocal.Request()
    request.north = north
    request.east = east
    request.altitude = altitude
    request.yaw_deg = yaw_deg
    result = bridge_node.call_service_blocking(bridge_node.goto_client, request, 70.0)
    if isinstance(result, str):
        return result
    if result.success:
        return f'arrived: {result.message}'
    return f'goto failed: {result.message}'


# Land the copter where it is and wait for disarm
@mcp.tool
def land() -> str:
    """Land the copter at its current position and wait for disarm."""
    if bridge_node is None:
        return 'bridge not started, launch the flight stack first'

    result = bridge_node.call_service_blocking(bridge_node.land_client, Trigger.Request(), 70.0)
    if isinstance(result, str):
        return result
    if result.success:
        return f'landed: {result.message}'
    return f'land failed: {result.message}'


# Force the safety guard into RTL, this latches
@mcp.tool
def abort() -> str:
    """Emergency stop, force return to launch, this latches and cannot be undone in flight."""
    if bridge_node is None:
        return 'bridge not started, launch the flight stack first'

    result = bridge_node.call_service_blocking(bridge_node.abort_client, Trigger.Request(), 15.0)
    if isinstance(result, str):
        return result
    if result.success:
        return f'abort triggered: {result.message}'
    return f'abort failed: {result.message}'


# Read the stored mode, armed, battery, and position without a service call
@mcp.tool
def get_telemetry() -> str:
    """Current mode, armed state, battery percent, and position in meters north, east, and altitude."""
    if bridge_node is None:
        return 'bridge not started, launch the flight stack first'

    state = bridge_node.state_message
    mode = state.mode if state is not None else 'unknown'
    armed = state.armed if state is not None else 'unknown'

    battery = bridge_node.battery_message
    battery_percent = f'{round(battery.percentage * 100)}%' if battery is not None else 'unknown'

    # Read side mirror of build_enu_setpoint, the commander stays the single
    # authority for the command side conversion
    pose = bridge_node.pose_message
    north = round(pose.pose.position.y, 2) if pose is not None else 'unknown'
    east = round(pose.pose.position.x, 2) if pose is not None else 'unknown'
    altitude = round(pose.pose.position.z, 2) if pose is not None else 'unknown'

    return (f'mode={mode} armed={armed} battery={battery_percent} '
            f'north={north} east={east} altitude={altitude}')


# Read the stored mission director state without a service call
@mcp.tool
def get_mission_state() -> str:
    """Current mission director state, IDLE through LANDED, unknown when no mission is running."""
    if bridge_node is None:
        return 'bridge not started, launch the flight stack first'
    if bridge_node.mission_state_message is None:
        return 'unknown'
    return bridge_node.mission_state_message.data


# List the finished report directories
@mcp.resource('kestrel://reports')
def list_reports() -> str:
    """Newline list of finished report directories under reports/."""
    if not os.path.isdir('reports'):
        return ''
    directories = sorted(
        name for name in os.listdir('reports')
        if os.path.isdir(os.path.join('reports', name)) and name != 'current')
    return '\n'.join(directories)


# Return the markdown content of one finished report
@mcp.resource('kestrel://reports/{timestamp}')
def get_report(timestamp: str) -> str:
    """Content of one finished report, or a not found message."""
    report_path = os.path.join('reports', timestamp, 'report.md')
    if not os.path.isfile(report_path):
        return 'no such report'
    with open(report_path) as report_file:
        return report_file.read()


# Start the rclpy node in a background thread, then run the MCP server
def main():
    global bridge_node

    rclpy.init()
    node = McpBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    bridge_node = node

    mcp.run()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

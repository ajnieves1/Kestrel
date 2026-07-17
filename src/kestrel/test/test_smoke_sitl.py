# SITL smoke test: cold start to hover and land, gated behind an env var
import os
import signal
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from kestrel_msgs.srv import Takeoff
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger

pytestmark = pytest.mark.skipif(
    os.environ.get('KESTREL_RUN_SITL_SMOKE') != '1',
    reason='set KESTREL_RUN_SITL_SMOKE=1 to run')


# Helper node that watches vehicle state and pose during the smoke test
class SmokeTestNode(Node):
    # Subscribe to state and pose with sensor QoS
    def __init__(self):
        super().__init__('smoke_test_node')
        self.state_message = None
        self.pose_message = None
        self.create_subscription(
            State, '/mavros/state', self.on_state, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.on_pose,
            qos_profile_sensor_data)

    # Store the latest state message
    def on_state(self, state_message):
        self.state_message = state_message

    # Store the latest pose message
    def on_pose(self, pose_message):
        self.pose_message = pose_message


# Cold start to hover then land, proves the full flight stack over MAVLink
@pytest.mark.timeout(300)
def test_takeoff_and_land():
    sitl_process = subprocess.Popen(
        ['ros2', 'launch', 'kestrel', 'sitl.launch.py', 'headless:=true'],
        start_new_session=True)
    commander_process = subprocess.Popen(
        ['ros2', 'run', 'kestrel', 'flight_commander'], start_new_session=True)

    rclpy.init()
    smoke_test_node = SmokeTestNode()

    try:
        # Wait for the FCU connection before calling any commander service
        connected = False
        deadline = time.time() + 120.0
        while time.time() < deadline:
            rclpy.spin_once(smoke_test_node, timeout_sec=0.1)
            if (smoke_test_node.state_message is not None
                    and smoke_test_node.state_message.connected):
                connected = True
                break
        assert connected, 'timed out waiting for FCU connection'

        takeoff_client = smoke_test_node.create_client(Takeoff, '/kestrel/cmd/takeoff')
        while not takeoff_client.wait_for_service(timeout_sec=1.0):
            pass
        takeoff_request = Takeoff.Request()
        takeoff_request.altitude = 5.0
        takeoff_future = takeoff_client.call_async(takeoff_request)
        rclpy.spin_until_future_complete(
            smoke_test_node, takeoff_future, timeout_sec=120.0)
        takeoff_result = takeoff_future.result()
        assert takeoff_result is not None and takeoff_result.success

        assert smoke_test_node.pose_message is not None
        assert abs(smoke_test_node.pose_message.pose.position.z - 5.0) <= 0.7

        land_client = smoke_test_node.create_client(Trigger, '/kestrel/cmd/land')
        while not land_client.wait_for_service(timeout_sec=1.0):
            pass
        land_future = land_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(smoke_test_node, land_future, timeout_sec=120.0)
        land_result = land_future.result()
        assert land_result is not None and land_result.success

        armed = smoke_test_node.state_message.armed
        deadline = time.time() + 30.0
        while armed and time.time() < deadline:
            rclpy.spin_once(smoke_test_node, timeout_sec=0.1)
            armed = smoke_test_node.state_message.armed
        assert not armed, 'timed out waiting for disarm after land'
    finally:
        os.killpg(os.getpgid(sitl_process.pid), signal.SIGTERM)
        os.killpg(os.getpgid(commander_process.pid), signal.SIGTERM)
        smoke_test_node.destroy_node()
        rclpy.shutdown()

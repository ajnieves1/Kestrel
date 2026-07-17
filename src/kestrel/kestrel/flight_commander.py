# Offer takeoff, goto, and land services that wrap MAVROS control of ArduCopter
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from kestrel_msgs.srv import GotoLocal, Takeoff
from mavros_msgs.msg import HomePosition, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from std_srvs.srv import Trigger

# Total time to keep retrying the arm command while ArduCopter's EKF settles
ARM_READY_DEADLINE_SECONDS = 120.0
# Pause between arm attempts, matches design.md A14's original spacing
ARM_RETRY_PAUSE_SECONDS = 3.0


# Offer takeoff, goto, and land services that wrap MAVROS control of ArduCopter
class FlightCommander(Node):
    # Create service servers, MAVROS clients, subscriptions, and state
    def __init__(self):
        super().__init__('flight_commander')

        # One reentrant group so a service call can wait inside a callback
        self.callback_group = ReentrantCallbackGroup()

        self.state_message = None
        self.pose_message = None
        self.home_received = False

        self.create_subscription(
            State, '/mavros/state', self.on_state,
            qos_profile_sensor_data, callback_group=self.callback_group)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.on_pose,
            qos_profile_sensor_data, callback_group=self.callback_group)
        # Home position is latched transient local, match it or a late start misses it
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(
            HomePosition, '/mavros/home_position/home', self.on_home,
            latched_qos, callback_group=self.callback_group)

        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode', callback_group=self.callback_group)
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming', callback_group=self.callback_group)
        self.takeoff_client = self.create_client(
            CommandTOL, '/mavros/cmd/takeoff', callback_group=self.callback_group)
        self.land_client = self.create_client(
            CommandTOL, '/mavros/cmd/land', callback_group=self.callback_group)

        self.setpoint_publisher = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        self.create_service(
            Takeoff, '/kestrel/cmd/takeoff', self.handle_takeoff,
            callback_group=self.callback_group)
        self.create_service(
            GotoLocal, '/kestrel/cmd/goto', self.handle_goto,
            callback_group=self.callback_group)
        self.create_service(
            Trigger, '/kestrel/cmd/land', self.handle_land,
            callback_group=self.callback_group)

    # Store the latest vehicle state message
    def on_state(self, state_message):
        self.state_message = state_message

    # Store the latest local position message
    def on_pose(self, pose_message):
        self.pose_message = pose_message

    # Note that the home position arrived, the EKF origin is now set
    def on_home(self, home_message):
        self.home_received = True

    # Handle /kestrel/cmd/takeoff by arming and climbing to the target altitude
    def handle_takeoff(self, request, response):
        target_altitude = request.altitude

        # Step 1: wait for the FCU connection
        connected = False
        deadline = time.time() + 30.0
        while time.time() < deadline:
            if self.state_message is not None and self.state_message.connected:
                connected = True
                break
            time.sleep(0.1)
        if not connected:
            response.success = False
            response.message = 'timed out waiting for FCU connection'
            return response

        # Step 2: wait for the home position, this means the EKF has an origin
        home_ready = False
        deadline = time.time() + 90.0
        while time.time() < deadline:
            if self.home_received:
                home_ready = True
                break
            time.sleep(0.1)
        if not home_ready:
            response.success = False
            response.message = 'timed out waiting for home position'
            return response

        # Step 3: switch to GUIDED and confirm through the state topic
        mode_request = SetMode.Request()
        mode_request.custom_mode = 'GUIDED'
        self.call_service_blocking(self.set_mode_client, mode_request, 5.0)
        in_guided = False
        deadline = time.time() + 10.0
        while time.time() < deadline:
            if self.state_message is not None and self.state_message.mode == 'GUIDED':
                in_guided = True
                break
            time.sleep(0.1)
        if not in_guided:
            response.success = False
            response.message = 'failed to enter GUIDED mode'
            return response

        # Step 4: arm, ArduPilot rejects until its prearm checks pass.
        # A14 originally spaced 5 attempts 3 s apart, but real flights showed
        # the EKF position estimate can take much longer than that to settle
        armed = False
        arm_deadline = time.time() + ARM_READY_DEADLINE_SECONDS
        while time.time() < arm_deadline:
            arm_request = CommandBool.Request()
            arm_request.value = True
            arm_result = self.call_service_blocking(self.arming_client, arm_request, 5.0)
            if arm_result is not None and arm_result.success:
                armed = True
                break
            time.sleep(ARM_RETRY_PAUSE_SECONDS)
        if not armed:
            response.success = False
            response.message = 'failed to arm within the retry window'
            return response

        # Step 5: command the takeoff to the target altitude
        takeoff_request = CommandTOL.Request()
        takeoff_request.altitude = target_altitude
        takeoff_result = self.call_service_blocking(
            self.takeoff_client, takeoff_request, 5.0)
        if takeoff_result is None or not takeoff_result.success:
            response.success = False
            response.message = 'takeoff command was rejected'
            return response

        # Step 6: wait until the altitude reaches the target
        reached = False
        deadline = time.time() + 30.0
        while time.time() < deadline:
            if self.pose_message is not None:
                altitude = self.pose_message.pose.position.z
                if abs(altitude - target_altitude) <= 0.5:
                    reached = True
                    break
            time.sleep(0.1)
        if not reached:
            response.success = False
            response.message = 'timed out climbing to the target altitude'
            return response

        response.success = True
        response.message = 'reached target altitude'
        return response

    # Handle /kestrel/cmd/goto by streaming setpoints until arrival or timeout
    def handle_goto(self, request, response):
        # Refuse unless armed and in GUIDED, setpoints are ignored otherwise
        if (self.state_message is None or not self.state_message.armed
                or self.state_message.mode != 'GUIDED'):
            response.success = False
            response.message = 'vehicle is not armed and in GUIDED'
            return response

        target = self.build_enu_setpoint(
            request.north, request.east, request.altitude, request.yaw_deg)

        # Stream the setpoint at 10 Hz until the pose is within half a meter
        arrived = False
        remaining = float('inf')
        deadline = time.time() + 60.0
        while time.time() < deadline:
            target.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_publisher.publish(target)
            if self.pose_message is not None:
                current = self.pose_message.pose.position
                remaining = math.sqrt(
                    (current.x - target.pose.position.x) ** 2
                    + (current.y - target.pose.position.y) ** 2
                    + (current.z - target.pose.position.z) ** 2)
                if remaining < 0.5:
                    arrived = True
                    break
            time.sleep(0.1)

        if not arrived:
            response.success = False
            response.message = f'timed out {remaining:.2f} m from the target'
            return response

        response.success = True
        response.message = 'reached the target'
        return response

    # Handle /kestrel/cmd/land by calling MAVROS land and waiting for disarm
    def handle_land(self, request, response):
        land_result = self.call_service_blocking(
            self.land_client, CommandTOL.Request(), 5.0)
        if land_result is None or not land_result.success:
            response.success = False
            response.message = 'land command was rejected'
            return response

        # Landing finishes when ArduPilot disarms the motors on touchdown
        disarmed = False
        deadline = time.time() + 60.0
        while time.time() < deadline:
            if self.state_message is not None and not self.state_message.armed:
                disarmed = True
                break
            time.sleep(0.1)
        if not disarmed:
            response.success = False
            response.message = 'timed out waiting for disarm after land'
            return response

        response.success = True
        response.message = 'landed and disarmed'
        return response

    # MAVROS local frame is ENU while our services speak north and east, convert here
    def build_enu_setpoint(self, north, east, altitude, yaw_deg):
        setpoint = PoseStamped()
        setpoint.pose.position.x = float(east)
        setpoint.pose.position.y = float(north)
        setpoint.pose.position.z = float(altitude)
        # Compass yaw is from north clockwise, ENU yaw is from east toward north
        yaw = math.radians(90.0 - yaw_deg)
        setpoint.pose.orientation.x = 0.0
        setpoint.pose.orientation.y = 0.0
        setpoint.pose.orientation.z = math.sin(yaw / 2.0)
        setpoint.pose.orientation.w = math.cos(yaw / 2.0)
        return setpoint

    # Call a MAVROS service and wait for the reply with a deadline
    def call_service_blocking(self, client, request, timeout_seconds):
        # Wait for the service to be available before calling it
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f'waiting for service {client.srv_name} to become available')

        future = client.call_async(request)

        # Poll the future rather than spin, this runs inside a service callback
        deadline = time.time() + timeout_seconds
        while time.time() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.05)

        self.get_logger().error(
            f'service {client.srv_name} did not respond within {timeout_seconds} s')
        return None


# Start the node and spin
def main():
    rclpy.init()
    flight_commander = FlightCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(flight_commander)
    executor.spin()
    flight_commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# Watch position and battery and force RTL when limits are broken
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger


# Watch position and battery and force RTL when limits are broken
class SafetyGuard(Node):
    # Subscribe to pose, battery, and state, offer the abort service
    def __init__(self):
        super().__init__('safety_guard')

        self.declare_parameter('geofence_radius_m', 100.0)
        self.declare_parameter('battery_floor_percent', 25.0)
        self.geofence_radius_m = self.get_parameter('geofence_radius_m').value
        self.battery_floor_percent = self.get_parameter('battery_floor_percent').value

        self.state_message = None
        self.pose_message = None
        self.battery_message = None
        self.rtl_latched = False

        self.create_subscription(
            State, '/mavros/state', self.on_state, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.on_pose,
            qos_profile_sensor_data)
        self.create_subscription(
            BatteryState, '/mavros/battery', self.on_battery, qos_profile_sensor_data)

        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.create_service(Trigger, '/kestrel/abort', self.handle_abort)

        self.create_timer(1.0, self.check_limits)

    # Store the latest vehicle state message
    def on_state(self, state_message):
        self.state_message = state_message

    # Store the latest local position message
    def on_pose(self, pose_message):
        self.pose_message = pose_message

    # Store the latest battery message
    def on_battery(self, battery_message):
        self.battery_message = battery_message

    # Handle /kestrel/abort by forcing RTL
    def handle_abort(self, request, response):
        self.trigger_rtl('abort service')
        response.success = True
        response.message = 'return to launch commanded'
        return response

    # Check fence and battery once per second and trigger RTL on breach
    def check_limits(self):
        if self.state_message is None:
            return

        # Once latched, keep commanding RTL until the FCU confirms the mode
        if self.rtl_latched:
            if self.state_message.mode != 'RTL':
                self.trigger_rtl('reasserting return to launch')
            return

        # An idle vehicle on the pad must not trigger the guard
        if not self.state_message.armed:
            return

        if self.pose_message is not None:
            x = self.pose_message.pose.position.x
            y = self.pose_message.pose.position.y
            if math.sqrt(x * x + y * y) > self.geofence_radius_m:
                self.trigger_rtl('geofence breach')
                return

        if self.battery_message is not None:
            if self.battery_message.percentage * 100 < self.battery_floor_percent:
                self.trigger_rtl('low battery')
                return

    # Set mode RTL through MAVROS and latch so we never fight the return
    def trigger_rtl(self, reason):
        # Log and latch on the first trip so the limit checks stop
        if not self.rtl_latched:
            self.get_logger().error(f'safety guard forcing RTL, reason: {reason}')
            self.rtl_latched = True

        # Command RTL, the timer calls this again until the FCU confirms it
        mode_request = SetMode.Request()
        mode_request.custom_mode = 'RTL'
        self.set_mode_client.call_async(mode_request)


# Start the node and spin
def main():
    rclpy.init()
    safety_guard = SafetyGuard()
    rclpy.spin(safety_guard)
    safety_guard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

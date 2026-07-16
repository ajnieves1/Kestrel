# Watch vehicle state and battery and log a one line summary each second
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState


# Watch vehicle state and battery and log a one line summary each second
class TelemetryMonitor(Node):
    # Subscribe to mavros state, battery, and local position
    def __init__(self):
        super().__init__('telemetry_monitor')
        self.state_message = None
        self.battery_message = None
        self.pose_message = None

        self.create_subscription(
            State, '/mavros/state', self.on_state, qos_profile_sensor_data)
        self.create_subscription(
            BatteryState, '/mavros/battery', self.on_battery, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.on_pose,
            qos_profile_sensor_data)

        self.create_timer(1.0, self.log_summary)

    # Store the latest vehicle state message
    def on_state(self, state_message):
        self.state_message = state_message

    # Store the latest battery message
    def on_battery(self, battery_message):
        self.battery_message = battery_message

    # Store the latest local position message
    def on_pose(self, pose_message):
        self.pose_message = pose_message

    # Log mode, armed flag, battery percent, and altitude
    def log_summary(self):
        mode = self.state_message.mode if self.state_message is not None else 'unknown'
        armed = self.state_message.armed if self.state_message is not None else 'unknown'
        if self.battery_message is not None:
            battery_percent = f'{round(self.battery_message.percentage * 100)}%'
        else:
            battery_percent = 'unknown'
        if self.pose_message is not None:
            altitude = f'{self.pose_message.pose.position.z:.2f}m'
        else:
            altitude = 'unknown'

        self.get_logger().info(
            f'mode={mode} armed={armed} battery={battery_percent} altitude={altitude}')


# Start the node and spin
def main():
    rclpy.init()
    telemetry_monitor = TelemetryMonitor()
    rclpy.spin(telemetry_monitor)
    telemetry_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

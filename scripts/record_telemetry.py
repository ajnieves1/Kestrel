#!/usr/bin/env python3
# Record propulsion and power telemetry to CSV while the vehicle is armed, for
# training the health monitor. Run alongside a flight, stop with Ctrl-C.
import math
import os
import time

import rclpy
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu

SAMPLE_RATE_HZ = 5.0
OUTPUT_DIRECTORY = 'telemetry_logs'


# Sample four health features at a fixed rate while armed and append them to CSV
class TelemetryRecorder(Node):
    def __init__(self):
        super().__init__('record_telemetry')

        self.declare_parameter('label', 'nominal')
        self.label = self.get_parameter('label').value

        self.start_time = time.time()
        self.battery_message = None
        self.imu_message = None
        self.state_message = None
        self.row_count = 0

        os.makedirs(OUTPUT_DIRECTORY, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.output_path = os.path.join(
            OUTPUT_DIRECTORY, f'{self.label}_{timestamp}.csv')
        self.output_file = open(self.output_path, 'w')
        self.output_file.write('t,voltage,current,ang_vel_mag,lin_acc_mag\n')
        self.get_logger().info(
            f'recording to {os.path.abspath(self.output_path)}, '
            'rows are written only while the vehicle is armed')

        self.create_subscription(
            BatteryState, '/mavros/battery', self.on_battery,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/mavros/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            State, '/mavros/state', self.on_state, qos_profile_sensor_data)

        self.create_timer(1.0 / SAMPLE_RATE_HZ, self.sample)
        self.create_timer(5.0, self.log_status)

    # Store the latest battery message
    def on_battery(self, battery_message):
        self.battery_message = battery_message

    # Store the latest IMU message
    def on_imu(self, imu_message):
        self.imu_message = imu_message

    # Store the latest vehicle state message
    def on_state(self, state_message):
        self.state_message = state_message

    # Write one feature row when armed and all inputs are present
    def sample(self):
        if self.state_message is None or not self.state_message.armed:
            return
        if self.battery_message is None or self.imu_message is None:
            return

        angular = self.imu_message.angular_velocity
        linear = self.imu_message.linear_acceleration
        ang_vel_mag = math.sqrt(
            angular.x ** 2 + angular.y ** 2 + angular.z ** 2)
        lin_acc_mag = math.sqrt(
            linear.x ** 2 + linear.y ** 2 + linear.z ** 2)

        elapsed = time.time() - self.start_time
        self.output_file.write(
            f'{elapsed:.3f},{self.battery_message.voltage:.4f},'
            f'{self.battery_message.current:.4f},{ang_vel_mag:.6f},'
            f'{lin_acc_mag:.6f}\n')
        self.output_file.flush()
        self.row_count += 1

    # Log whether recording is active, or which inputs are still missing
    def log_status(self):
        armed = self.state_message is not None and self.state_message.armed
        have_battery = self.battery_message is not None
        have_imu = self.imu_message is not None
        if armed and have_battery and have_imu:
            self.get_logger().info(f'recording, {self.row_count} rows so far')
        else:
            self.get_logger().warn(
                f'waiting to record: armed={armed} battery={have_battery} '
                f'imu={have_imu}')

    # Close the CSV file
    def close(self):
        self.output_file.close()
        self.get_logger().info(
            f'wrote {self.row_count} rows to {self.output_path}')


# Start the node and spin until interrupted, then close the file
def main():
    rclpy.init()
    recorder = TelemetryRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

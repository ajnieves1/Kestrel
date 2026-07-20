# Watch propulsion telemetry and raise a health alert on a sustained anomaly
import collections
import json
import math

import numpy as np
import onnxruntime
import rclpy
from kestrel_msgs.msg import HealthAlert
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu

# The four features the recorder captures, in a fixed order
FEATURE_COLUMNS = ['voltage', 'current', 'ang_vel_mag', 'lin_acc_mag']


# Watch propulsion telemetry and raise a health alert on a sustained anomaly
class HealthMonitor(Node):
    # Load the model and scaler, subscribe telemetry, set up the alert publisher
    def __init__(self):
        super().__init__('health_monitor')

        self.declare_parameter('model_path', 'models/health/health_model.onnx')
        self.declare_parameter('scaler_path', 'models/health/health_scaler.json')
        self.declare_parameter('consecutive_alerts_required', 8)

        self.model_path = self.get_parameter('model_path').value
        self.scaler_path = self.get_parameter('scaler_path').value
        self.consecutive_alerts_required = self.get_parameter(
            'consecutive_alerts_required').value

        # Window size, scaling, and threshold come from the trained model, so the
        # node can never disagree with what the model was trained on
        with open(self.scaler_path) as scaler_file:
            scaler = json.load(scaler_file)
        self.feature_names = scaler['feature_names']
        self.means = np.array(scaler['means'], dtype=np.float32)
        self.stds = np.array(scaler['stds'], dtype=np.float32)
        self.window_size = scaler['window_size']
        self.sample_rate_hz = scaler['sample_rate_hz']
        self.threshold = scaler['threshold']
        self.active_index = [FEATURE_COLUMNS.index(name) for name in self.feature_names]

        self.onnx_session = onnxruntime.InferenceSession(
            self.model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.onnx_session.get_inputs()[0].name

        self.window = collections.deque(maxlen=self.window_size)
        self.over_threshold_count = 0
        self.alerted = False

        self.battery_message = None
        self.imu_message = None
        self.state_message = None

        self.create_subscription(
            BatteryState, '/mavros/battery', self.on_battery, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/mavros/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            State, '/mavros/state', self.on_state, qos_profile_sensor_data)

        self.alert_publisher = self.create_publisher(
            HealthAlert, '/kestrel/health_alerts', 10)

        self.create_timer(1.0 / self.sample_rate_hz, self.check_health)

    # Store the latest battery message
    def on_battery(self, battery_message):
        self.battery_message = battery_message

    # Store the latest IMU message
    def on_imu(self, imu_message):
        self.imu_message = imu_message

    # Store the latest vehicle state message
    def on_state(self, state_message):
        self.state_message = state_message

    # Build the active feature vector from the latest telemetry
    def current_features(self):
        angular = self.imu_message.angular_velocity
        linear = self.imu_message.linear_acceleration
        full = [
            self.battery_message.voltage,
            self.battery_message.current,
            math.sqrt(angular.x ** 2 + angular.y ** 2 + angular.z ** 2),
            math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2),
        ]
        return np.array([full[i] for i in self.active_index], dtype=np.float32)

    # Score one telemetry window and raise an alert on a sustained anomaly
    def check_health(self):
        if self.alerted:
            return
        if self.state_message is None or not self.state_message.armed:
            # Reset between flights so a settled pad does not carry old state
            self.window.clear()
            self.over_threshold_count = 0
            return
        if self.battery_message is None or self.imu_message is None:
            return

        self.window.append(self.current_features())
        if len(self.window) < self.window_size:
            return

        standardized = (np.array(self.window) - self.means) / self.stds
        model_input = standardized.flatten().astype(np.float32)[None, :]
        reconstruction = self.onnx_session.run(
            None, {self.input_name: model_input})[0]
        error = float(((reconstruction - model_input) ** 2).mean())

        if error > self.threshold:
            self.over_threshold_count += 1
        else:
            self.over_threshold_count = 0

        if self.over_threshold_count >= self.consecutive_alerts_required:
            self.raise_alert(error)

    # Publish and log one propulsion health alert, then latch for the mission
    def raise_alert(self, error):
        self.alerted = True
        alert = HealthAlert()
        alert.header.stamp = self.get_clock().now().to_msg()
        alert.component = 'propulsion'
        alert.anomaly_score = float(error)
        alert.threshold = float(self.threshold)
        alert.message = (
            'sustained propulsion anomaly, possible motor degradation, '
            'recommend inspection')
        self.alert_publisher.publish(alert)
        self.get_logger().warn(
            f'health alert: propulsion anomaly score {error:.3f} over '
            f'threshold {self.threshold:.3f}')


# Start the node and spin
def main():
    rclpy.init()
    health_monitor = HealthMonitor()
    rclpy.spin(health_monitor)
    health_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

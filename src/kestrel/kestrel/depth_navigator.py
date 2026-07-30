# Estimate monocular depth from the camera and recommend a safer heading
import time

import numpy as np
import onnxruntime
import rclpy
from cv_bridge import CvBridge
import cv2
from kestrel_msgs.msg import NavigationAdvisory
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

INPUT_SIZE = 256
# ImageNet normalization, must match scripts/export_depth_model.py
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


# Resize to the model input and normalize into an NCHW float batch
def preprocess(image_bgr):
    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    resized = cv2.resize(
        rgb, (INPUT_SIZE, INPUT_SIZE), interpolation=cv2.INTER_CUBIC)
    normalized = (resized - MEAN) / STD
    return np.transpose(normalized, (2, 0, 1))[None].astype(np.float32)


# Split the middle band into columns and pick the clearer heading
# Depth is inverse, larger means closer. Returns hazard, heading, and proximities
def analyze_depth(depth_map, hazard_ratio):
    height, width = depth_map.shape
    # Use the middle rows, the flight path, so ground and sky do not dominate
    band = depth_map[height // 4:3 * height // 4]
    third = width // 3
    left = float(band[:, :third].mean())
    center = float(band[:, third:2 * third].mean())
    right = float(band[:, 2 * third:].mean())
    clearer_side = 'left' if left <= right else 'right'
    clearest_proximity = min(left, right)
    hazard = center > hazard_ratio * clearest_proximity
    return hazard, clearer_side, center, clearest_proximity


# Estimate monocular depth from the camera and recommend a safer heading
class DepthNavigator(Node):
    # Load the depth model, subscribe the camera and state, set up the publisher
    def __init__(self):
        super().__init__('depth_navigator')

        self.declare_parameter('model_path', 'models/depth/midas_small.onnx')
        self.declare_parameter('inference_rate_hz', 2.0)
        self.declare_parameter('hazard_ratio', 1.3)
        self.declare_parameter('consecutive_alerts_required', 2)

        self.model_path = self.get_parameter('model_path').value
        self.inference_rate_hz = self.get_parameter('inference_rate_hz').value
        self.hazard_ratio = self.get_parameter('hazard_ratio').value
        self.consecutive_alerts_required = self.get_parameter(
            'consecutive_alerts_required').value

        self.session = onnxruntime.InferenceSession(
            self.model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.bridge = CvBridge()

        self.state_message = None
        self.last_process_time = 0.0
        self.over_count = 0
        self.in_hazard = False

        self.create_subscription(
            Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            State, '/mavros/state', self.on_state, qos_profile_sensor_data)

        self.advisory_publisher = self.create_publisher(
            NavigationAdvisory, '/kestrel/nav_advisories', 10)

    # Store the latest vehicle state message
    def on_state(self, state_message):
        self.state_message = state_message

    # Run depth on one frame and advise a heading when structure is close ahead
    def on_image(self, image_message):
        if self.state_message is None or not self.state_message.armed:
            return
        now = time.time()
        if now - self.last_process_time < 1.0 / self.inference_rate_hz:
            return
        self.last_process_time = now

        cv_image = self.bridge.imgmsg_to_cv2(image_message, 'bgr8')
        batch = preprocess(cv_image)
        depth = self.session.run(None, {self.input_name: batch})[0].squeeze()
        hazard, heading, center, clearest = analyze_depth(
            depth, self.hazard_ratio)

        if not hazard:
            self.over_count = 0
            self.in_hazard = False
            return

        self.over_count += 1
        if self.over_count >= self.consecutive_alerts_required and not self.in_hazard:
            self.in_hazard = True
            self.raise_advisory(heading, center, clearest)

    # Publish and log one navigation advisory
    def raise_advisory(self, heading, center, clearest):
        advisory = NavigationAdvisory()
        advisory.header.stamp = self.get_clock().now().to_msg()
        advisory.hazard = True
        advisory.recommended_heading = heading
        advisory.center_proximity = float(center)
        advisory.clearest_proximity = float(clearest)
        advisory.message = (
            f'structure close ahead, more clearance to the {heading}, '
            'recommend veering')
        self.advisory_publisher.publish(advisory)
        self.get_logger().warn(
            f'navigation advisory: structure ahead, veer {heading}')


# Start the node and spin
def main():
    rclpy.init()
    depth_navigator = DepthNavigator()
    rclpy.spin(depth_navigator)
    depth_navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

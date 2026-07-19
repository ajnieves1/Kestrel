# Detect defect markers on the camera stream and fire defect events
import math
import os
import time

import cv2
import numpy as np
import onnxruntime
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from kestrel_msgs.msg import DefectEvent
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

YOLO_INPUT_SIZE = 640
YOLO_NMS_IOU_THRESHOLD = 0.45


# Detect defect markers on the camera stream and fire defect events
class DefectDetector(Node):
    # Subscribe to camera image and info, set up the detector and publishers
    def __init__(self):
        super().__init__('defect_detector')

        self.declare_parameter('marker_size_m', 0.5)
        self.declare_parameter('inference_rate_hz', 5.0)
        self.declare_parameter('consecutive_frames_required', 5)
        self.declare_parameter('photo_directory', 'reports/current/photos')
        self.declare_parameter('detector_backend', 'yolo')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('dedupe_radius_m', 3.0)
        self.declare_parameter('model_path', 'models/vision/defect.onnx')
        self.declare_parameter('assumed_depth_m', 6.5)

        self.marker_size_m = self.get_parameter('marker_size_m').value
        self.inference_rate_hz = self.get_parameter('inference_rate_hz').value
        self.consecutive_frames_required = self.get_parameter(
            'consecutive_frames_required').value
        self.photo_directory = self.get_parameter('photo_directory').value
        self.detector_backend = self.get_parameter('detector_backend').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.dedupe_radius_m = self.get_parameter('dedupe_radius_m').value
        self.model_path = self.get_parameter('model_path').value
        self.assumed_depth_m = self.get_parameter('assumed_depth_m').value

        # Object points for solvePnP, matching the detectMarkers corner order
        # (top left, top right, bottom right, bottom left)
        half_size = self.marker_size_m / 2.0
        self.marker_object_points = np.array([
            [-half_size, half_size, 0.0],
            [half_size, half_size, 0.0],
            [half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ], dtype=np.float32)

        self.bridge = CvBridge()
        # ArucoDetector needs OpenCV 4.7, the image pins 4.6, use the old functional API
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        # Default adaptive threshold window is tuned for larger markers, our
        # marker is roughly 50 px in frame, a narrower window finds it reliably
        self.aruco_parameters.adaptiveThreshWinSizeMin = 5
        self.aruco_parameters.adaptiveThreshWinSizeMax = 15
        self.aruco_parameters.adaptiveThreshWinSizeStep = 2

        self.onnx_session = None
        self.onnx_input_name = None
        if self.detector_backend == 'yolo':
            self.onnx_session = onnxruntime.InferenceSession(
                self.model_path, providers=['CPUExecutionProvider'])
            self.onnx_input_name = self.onnx_session.get_inputs()[0].name

        self.camera_matrix = None
        self.dist_coeffs = None
        self.pose_message = None
        self.last_processed_time = 0.0
        # ArUco backend tracks debounce by marker id, YOLO by position cluster
        self.consecutive_counts = {}
        self.fired_marker_ids = set()
        self.tracked_positions = []

        self.create_subscription(
            Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_camera_info,
            qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.on_pose,
            qos_profile_sensor_data)

        self.detections_publisher = self.create_publisher(
            Detection2DArray, '/kestrel/detections', 10)
        self.events_publisher = self.create_publisher(
            DefectEvent, '/kestrel/defect_events', 10)

    # Store camera intrinsics once
    def on_camera_info(self, camera_info_message):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(camera_info_message.k).reshape(3, 3)
            self.dist_coeffs = np.array(camera_info_message.d)

    # Store the latest vehicle pose
    def on_pose(self, pose_message):
        self.pose_message = pose_message

    # Detect markers, publish detections, debounce and fire events
    def on_image(self, image_message):
        now = time.time()
        if now - self.last_processed_time < 1.0 / self.inference_rate_hz:
            return
        self.last_processed_time = now

        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='bgr8')

        if self.detector_backend == 'aruco':
            self.process_aruco_frame(cv_image, image_message)
        else:
            self.process_yolo_frame(cv_image, image_message)

    # Detect ArUco markers, publish detections, debounce and fire events by id
    def process_aruco_frame(self, cv_image, image_message):
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)

        detection_array = Detection2DArray()
        detection_array.header = image_message.header
        corners_by_id = {}
        if ids is not None:
            for index in range(len(ids)):
                marker_id = int(ids[index][0])
                marker_corners = corners[index][0]
                corners_by_id[marker_id] = marker_corners

                detection = Detection2D()
                detection.header = image_message.header
                min_x = float(marker_corners[:, 0].min())
                max_x = float(marker_corners[:, 0].max())
                min_y = float(marker_corners[:, 1].min())
                max_y = float(marker_corners[:, 1].max())
                detection.bbox.center.position.x = (min_x + max_x) / 2.0
                detection.bbox.center.position.y = (min_y + max_y) / 2.0
                detection.bbox.size_x = max_x - min_x
                detection.bbox.size_y = max_y - min_y
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = f'marker_{marker_id}'
                hypothesis.hypothesis.score = 1.0
                detection.results.append(hypothesis)
                detection_array.detections.append(detection)

        self.detections_publisher.publish(detection_array)

        # Can not localize a marker without intrinsics or a current pose
        if self.camera_matrix is None or self.pose_message is None:
            return

        detected_ids = set(corners_by_id.keys())
        tracked_ids = set(self.consecutive_counts.keys()) | detected_ids
        for marker_id in tracked_ids:
            if marker_id in detected_ids:
                self.consecutive_counts[marker_id] = (
                    self.consecutive_counts.get(marker_id, 0) + 1)
            else:
                self.consecutive_counts[marker_id] = 0

            if (self.consecutive_counts[marker_id] < self.consecutive_frames_required
                    or marker_id in self.fired_marker_ids):
                continue

            image_points = corners_by_id[marker_id].astype(np.float32)
            solved, _, translation_vector = cv2.solvePnP(
                self.marker_object_points, image_points,
                self.camera_matrix, self.dist_coeffs)
            if not solved:
                continue

            label = f'marker_{marker_id}'
            north, east, altitude = self.estimate_world_position(translation_vector)
            image_path = self.save_photo(cv_image, label)
            self.publish_defect_event(image_message, label, 1.0, north, east, altitude, image_path)

            self.fired_marker_ids.add(marker_id)

    # Run the YOLO backend, publish detections, debounce and fire events by position
    def process_yolo_frame(self, cv_image, image_message):
        yolo_detections = self.run_yolo(cv_image)

        detection_array = Detection2DArray()
        detection_array.header = image_message.header
        for center_x, center_y, width, height, confidence in yolo_detections:
            detection = Detection2D()
            detection.header = image_message.header
            detection.bbox.center.position.x = center_x
            detection.bbox.center.position.y = center_y
            detection.bbox.size_x = width
            detection.bbox.size_y = height
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'corrosion'
            hypothesis.hypothesis.score = confidence
            detection.results.append(hypothesis)
            detection_array.detections.append(detection)
        self.detections_publisher.publish(detection_array)

        # Can not localize a detection without intrinsics or a current pose
        if self.camera_matrix is None or self.pose_message is None:
            return

        matched_indices = set()
        for center_x, center_y, _, _, confidence in yolo_detections:
            translation_vector = self.build_ray_translation_vector(center_x, center_y)
            north, east, altitude = self.estimate_world_position(translation_vector)

            nearest_index = None
            nearest_distance = self.dedupe_radius_m
            for index, tracked in enumerate(self.tracked_positions):
                distance = self.distance_meters(
                    (north, east, altitude), tracked['position'])
                if distance <= nearest_distance:
                    nearest_distance = distance
                    nearest_index = index

            if nearest_index is None:
                self.tracked_positions.append({
                    'position': (north, east, altitude),
                    'confidence': confidence,
                    'consecutive_count': 1,
                    'fired': False,
                })
                matched_indices.add(len(self.tracked_positions) - 1)
            else:
                matched_indices.add(nearest_index)
                tracked = self.tracked_positions[nearest_index]
                if not tracked['fired']:
                    tracked['consecutive_count'] += 1
                    tracked['position'] = (north, east, altitude)
                    tracked['confidence'] = confidence

        # A tracked position with no match this frame breaks its streak
        for index, tracked in enumerate(self.tracked_positions):
            if index not in matched_indices and not tracked['fired']:
                tracked['consecutive_count'] = 0

        for tracked in self.tracked_positions:
            if tracked['fired'] or tracked['consecutive_count'] < self.consecutive_frames_required:
                continue

            north, east, altitude = tracked['position']
            image_path = self.save_photo(cv_image, 'corrosion')
            self.publish_defect_event(
                image_message, 'corrosion', tracked['confidence'],
                north, east, altitude, image_path)
            tracked['fired'] = True

    # Run the ONNX model on one frame and return original pixel space boxes
    def run_yolo(self, cv_image):
        letterboxed, scale, pad_left, pad_top = self.letterbox_image(cv_image)
        rgb_image = cv2.cvtColor(letterboxed, cv2.COLOR_BGR2RGB)
        normalized = rgb_image.astype(np.float32) / 255.0
        input_tensor = np.transpose(normalized, (2, 0, 1))[np.newaxis, :]

        outputs = self.onnx_session.run(None, {self.onnx_input_name: input_tensor})
        # YOLOv8 export shape is (1, 4 + classes, anchors), transpose to (anchors, 4 + classes)
        predictions = outputs[0][0].T

        boxes = []
        scores = []
        for prediction in predictions:
            confidence = float(prediction[4:].max())
            if confidence < self.confidence_threshold:
                continue
            center_x, center_y, width, height = prediction[:4]
            boxes.append([
                float(center_x - width / 2), float(center_y - height / 2),
                float(width), float(height)])
            scores.append(confidence)

        if not boxes:
            return []

        kept_indices = cv2.dnn.NMSBoxes(
            boxes, scores, self.confidence_threshold, YOLO_NMS_IOU_THRESHOLD)

        results = []
        for index in np.array(kept_indices).flatten():
            x, y, width, height = boxes[index]
            # Undo the letterbox scale and padding to reach original pixel space
            x = (x - pad_left) / scale
            y = (y - pad_top) / scale
            width = width / scale
            height = height / scale
            results.append((x + width / 2, y + height / 2, width, height, scores[index]))

        return results

    # Resize and pad a frame to a square YOLO input, keeping the aspect ratio
    def letterbox_image(self, cv_image):
        height, width = cv_image.shape[:2]
        scale = min(YOLO_INPUT_SIZE / height, YOLO_INPUT_SIZE / width)
        new_height, new_width = round(height * scale), round(width * scale)
        resized = cv2.resize(cv_image, (new_width, new_height))

        pad_left = (YOLO_INPUT_SIZE - new_width) // 2
        pad_top = (YOLO_INPUT_SIZE - new_height) // 2
        pad_right = YOLO_INPUT_SIZE - new_width - pad_left
        pad_bottom = YOLO_INPUT_SIZE - new_height - pad_top
        padded = cv2.copyMakeBorder(
            resized, pad_top, pad_bottom, pad_left, pad_right,
            cv2.BORDER_CONSTANT, value=(114, 114, 114))

        return padded, scale, pad_left, pad_top

    # Build a translation vector as a ray through the bbox center at a fixed depth
    def build_ray_translation_vector(self, center_x, center_y):
        focal_x = self.camera_matrix[0, 0]
        focal_y = self.camera_matrix[1, 1]
        principal_x = self.camera_matrix[0, 2]
        principal_y = self.camera_matrix[1, 2]

        depth = self.assumed_depth_m
        x_optical = (center_x - principal_x) / focal_x * depth
        y_optical = (center_y - principal_y) / focal_y * depth
        return np.array([x_optical, y_optical, depth])

    # Straight line distance between two north east altitude points
    def distance_meters(self, point_a, point_b):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(point_a, point_b)))

    # Fill and publish one defect event message
    def publish_defect_event(self, image_message, label, confidence, north, east, altitude, image_path):
        event = DefectEvent()
        event.header = image_message.header
        event.label = label
        event.confidence = confidence
        event.world_position.x = north
        event.world_position.y = east
        event.world_position.z = altitude
        event.image_path = image_path
        self.events_publisher.publish(event)

    # Estimate the marker world position from vehicle pose and marker pose
    def estimate_world_position(self, translation_vector):
        x_optical = float(translation_vector[0])
        y_optical = float(translation_vector[1])
        z_optical = float(translation_vector[2])

        # Body frame (x forward, y left, z up), camera is 0.2 m forward and level
        x_body = z_optical + 0.2
        y_body = -x_optical
        z_body = -y_optical

        orientation = self.pose_message.pose.orientation
        # ENU yaw from the pose quaternion, measured from east toward north
        yaw = math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z))

        vehicle_position = self.pose_message.pose.position
        east = vehicle_position.x + x_body * math.cos(yaw) - y_body * math.sin(yaw)
        north = vehicle_position.y + x_body * math.sin(yaw) + y_body * math.cos(yaw)
        altitude = vehicle_position.z + z_body

        return north, east, altitude

    # Save the frame that confirmed a defect and return the file path
    def save_photo(self, cv_image, label):
        os.makedirs(self.photo_directory, exist_ok=True)
        filename = f'defect_{label}_{int(time.time())}.jpg'
        output_path = os.path.join(self.photo_directory, filename)
        cv2.imwrite(output_path, cv_image)
        return output_path


# Start the node and spin
def main():
    rclpy.init()
    defect_detector = DefectDetector()
    rclpy.spin(defect_detector)
    defect_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

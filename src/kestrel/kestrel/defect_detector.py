# Detect ArUco defect markers on the camera stream and fire defect events
import math
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from kestrel_msgs.msg import DefectEvent
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


# Detect ArUco defect markers on the camera stream and fire defect events
class DefectDetector(Node):
    # Subscribe to camera image and info, set up the detector and publishers
    def __init__(self):
        super().__init__('defect_detector')

        self.declare_parameter('marker_size_m', 0.5)
        self.declare_parameter('inference_rate_hz', 5.0)
        self.declare_parameter('consecutive_frames_required', 5)
        self.declare_parameter('photo_directory', 'reports/current/photos')

        self.marker_size_m = self.get_parameter('marker_size_m').value
        self.inference_rate_hz = self.get_parameter('inference_rate_hz').value
        self.consecutive_frames_required = self.get_parameter(
            'consecutive_frames_required').value
        self.photo_directory = self.get_parameter('photo_directory').value

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

        self.camera_matrix = None
        self.dist_coeffs = None
        self.pose_message = None
        self.last_processed_time = 0.0
        self.consecutive_counts = {}
        self.fired_marker_ids = set()

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

            north, east, altitude = self.estimate_world_position(translation_vector)
            image_path = self.save_photo(cv_image, marker_id)

            event = DefectEvent()
            event.header = image_message.header
            event.label = f'marker_{marker_id}'
            event.confidence = 1.0
            event.world_position.x = north
            event.world_position.y = east
            event.world_position.z = altitude
            event.image_path = image_path
            self.events_publisher.publish(event)

            self.fired_marker_ids.add(marker_id)

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
    def save_photo(self, cv_image, marker_id):
        os.makedirs(self.photo_directory, exist_ok=True)
        filename = f'defect_marker_{marker_id}_{int(time.time())}.jpg'
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

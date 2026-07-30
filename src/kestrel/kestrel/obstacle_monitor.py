# Watch the lidar point cloud and warn the operator on a close obstacle
import math

import numpy as np
import rclpy
from kestrel_msgs.msg import ObstacleHazard
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


# Nearest finite obstacle in a lidar cloud, fused to the world frame
# Points are in the lidar frame, x forward, y left, z up. The pose is ENU,
# x east, y north, z up. Returns distance, bearing, and world north east altitude
def nearest_obstacle(points, pose_east, pose_north, pose_altitude, yaw_rad):
    finite = points[np.isfinite(points).all(axis=1)]
    if len(finite) == 0:
        return None
    ranges = np.linalg.norm(finite, axis=1)
    index = int(np.argmin(ranges))
    x_forward, y_left, z_up = (float(v) for v in finite[index])
    distance = float(ranges[index])
    # Bearing from body forward, positive to the right
    bearing_deg = math.degrees(math.atan2(-y_left, x_forward))
    # World ENU from the vehicle pose and yaw, the same rule the detector uses
    east = pose_east + x_forward * math.cos(yaw_rad) - y_left * math.sin(yaw_rad)
    north = pose_north + x_forward * math.sin(yaw_rad) + y_left * math.cos(yaw_rad)
    altitude = pose_altitude + z_up
    return distance, bearing_deg, north, east, altitude


# ENU yaw in radians from a pose quaternion, measured from east toward north
def yaw_from_quaternion(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))


# Watch the lidar point cloud and warn the operator on a close obstacle
class ObstacleMonitor(Node):
    # Subscribe to the cloud, pose, and state, set up the hazard publisher
    def __init__(self):
        super().__init__('obstacle_monitor')

        self.declare_parameter('hazard_distance_m', 4.0)
        self.declare_parameter('consecutive_alerts_required', 3)
        self.hazard_distance_m = self.get_parameter('hazard_distance_m').value
        self.consecutive_alerts_required = self.get_parameter(
            'consecutive_alerts_required').value

        self.pose_message = None
        self.state_message = None
        self.over_count = 0
        self.in_hazard = False

        self.create_subscription(
            PointCloud2, '/lidar/points', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.on_pose,
            qos_profile_sensor_data)
        self.create_subscription(
            State, '/mavros/state', self.on_state, qos_profile_sensor_data)

        self.hazard_publisher = self.create_publisher(
            ObstacleHazard, '/kestrel/obstacle_hazards', 10)

    # Store the latest local position message
    def on_pose(self, pose_message):
        self.pose_message = pose_message

    # Store the latest vehicle state message
    def on_state(self, state_message):
        self.state_message = state_message

    # Score one cloud, fire a hazard when an obstacle is closer than the limit
    def on_cloud(self, cloud_message):
        if self.state_message is None or not self.state_message.armed:
            return
        if self.pose_message is None:
            return

        points = np.array(
            [[x, y, z] for x, y, z in point_cloud2.read_points(
                cloud_message, field_names=['x', 'y', 'z'], skip_nans=True)],
            dtype=np.float32)
        if len(points) == 0:
            return

        position = self.pose_message.pose.position
        yaw = yaw_from_quaternion(self.pose_message.pose.orientation)
        nearest = nearest_obstacle(
            points, position.x, position.y, position.z, yaw)
        if nearest is None:
            return
        distance, bearing_deg, north, east, altitude = nearest

        if distance >= self.hazard_distance_m:
            # Clear of the limit, re-arm so a later approach can fire again
            self.over_count = 0
            self.in_hazard = False
            return

        self.over_count += 1
        if self.over_count >= self.consecutive_alerts_required and not self.in_hazard:
            self.in_hazard = True
            self.raise_hazard(distance, bearing_deg, north, east, altitude)

    # Publish and log one obstacle proximity advisory
    def raise_hazard(self, distance, bearing_deg, north, east, altitude):
        hazard = ObstacleHazard()
        hazard.header.stamp = self.get_clock().now().to_msg()
        hazard.min_distance = float(distance)
        hazard.bearing_deg = float(bearing_deg)
        hazard.world_position.x = float(north)
        hazard.world_position.y = float(east)
        hazard.world_position.z = float(altitude)
        hazard.message = (
            f'obstacle {distance:.1f} m at bearing {bearing_deg:.0f} deg, '
            'closer than the standoff limit, recommend backing off')
        self.hazard_publisher.publish(hazard)
        self.get_logger().warn(
            f'obstacle hazard: {distance:.1f} m at bearing {bearing_deg:.0f} deg')


# Start the node and spin
def main():
    rclpy.init()
    obstacle_monitor = ObstacleMonitor()
    rclpy.spin(obstacle_monitor)
    obstacle_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

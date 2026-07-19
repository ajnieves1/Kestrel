#!/usr/bin/env python3
# Record one real mission's flight path, defect events, and state changes
# for the docs/ replay viewer. Run alongside a live mission, exits on LANDED.
import json
import os
import shutil
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from kestrel_msgs.msg import DefectEvent
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

POSE_RATE_HZ = 2.0
OUTPUT_DIRECTORY = 'docs/replay_data'


# Buffer pose, defect events, and state changes, write a replay JSON on LANDED
class ReplayRecorder(Node):
    def __init__(self):
        super().__init__('record_replay')

        self.declare_parameter('site_name', 'pylon')
        self.site_name = self.get_parameter('site_name').value

        self.start_time = time.time()
        self.last_pose_time = -1.0 / POSE_RATE_HZ
        self.poses = []
        self.events = []
        self.states = []
        self.last_state = None
        self.done = False

        self.timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.photo_directory = os.path.join(
            OUTPUT_DIRECTORY, f'{self.site_name}_{self.timestamp}_photos')

        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.on_pose,
            qos_profile_sensor_data)
        self.create_subscription(
            DefectEvent, '/kestrel/defect_events', self.on_defect_event,
            qos_profile_sensor_data)
        self.create_subscription(
            String, '/kestrel/mission_state', self.on_mission_state,
            qos_profile_sensor_data)

    def elapsed(self):
        return time.time() - self.start_time

    # Buffer a pose sample, throttled to POSE_RATE_HZ. ENU: x=east, y=north
    def on_pose(self, pose_message):
        now = self.elapsed()
        if now - self.last_pose_time < 1.0 / POSE_RATE_HZ:
            return
        self.last_pose_time = now
        position = pose_message.pose.position
        self.poses.append([
            round(now, 2), round(position.y, 2), round(position.x, 2),
            round(position.z, 2)])

    # Buffer a defect event and copy its photo next to the replay JSON
    def on_defect_event(self, defect_event):
        os.makedirs(self.photo_directory, exist_ok=True)
        photo_filename = os.path.basename(defect_event.image_path)
        if os.path.isfile(defect_event.image_path):
            shutil.copy(
                defect_event.image_path,
                os.path.join(self.photo_directory, photo_filename))

        position = defect_event.world_position
        self.events.append({
            't': round(self.elapsed(), 2),
            'label': defect_event.label,
            'confidence': defect_event.confidence,
            'north': round(position.x, 2),
            'east': round(position.y, 2),
            'altitude': round(position.z, 2),
            'photo': photo_filename,
        })

    # Buffer each state change, write the replay file and exit on LANDED
    def on_mission_state(self, state_message):
        if state_message.data == self.last_state:
            return
        self.last_state = state_message.data
        self.states.append([round(self.elapsed(), 2), state_message.data])

        if state_message.data == 'LANDED':
            self.write_replay_file()
            self.done = True

    # Write the recorded mission to a replay JSON the viewer can load
    def write_replay_file(self):
        os.makedirs(OUTPUT_DIRECTORY, exist_ok=True)
        record = {
            'site': self.site_name,
            'timestamp': self.timestamp,
            'duration_seconds': round(self.elapsed(), 2),
            'poses': self.poses,
            'events': self.events,
            'states': self.states,
        }
        output_path = os.path.join(
            OUTPUT_DIRECTORY, f'{self.site_name}_{self.timestamp}.json')
        with open(output_path, 'w') as output_file:
            json.dump(record, output_file, indent=2)

        self.get_logger().info(f'replay written to {output_path}')


# Start the node, spin until LANDED, then exit
def main():
    rclpy.init()
    recorder = ReplayRecorder()
    while rclpy.ok() and not recorder.done:
        rclpy.spin_once(recorder, timeout_sec=0.5)
    recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

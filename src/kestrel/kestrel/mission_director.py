# Run one full inspection as a state machine over the commander services
import collections
import threading
import time

import rclpy
from kestrel_msgs.msg import DefectEvent
from kestrel_msgs.srv import GotoLocal, Takeoff
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from kestrel.inspection_planner import (MINIMUM_ALTITUDE, Waypoint,
                                        build_orbit_path, build_survey_path)

# Total time to keep retrying takeoff while ArduCopter finishes prearm checks
TAKEOFF_READY_DEADLINE_SECONDS = 120.0
# Pause between takeoff retries so short lived prearm failures have time to clear
TAKEOFF_RETRY_PAUSE_SECONDS = 5.0
# Extra altitude above the structure height used to transit in over the top
SAFE_TRANSIT_CLEARANCE_METERS = 2.0


# Run one full inspection as a state machine over the commander services
class MissionDirector(Node):
    # Set up clients, subscriptions, state publisher, and the mission thread
    def __init__(self):
        super().__init__('mission_director')

        # One reentrant group so the mission thread can wait on service calls
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter('structure_center_north', 15.0)
        self.declare_parameter('structure_center_east', 0.0)
        self.declare_parameter('structure_height', 22.0)
        self.declare_parameter('survey_orbit_radius', 8.0)
        self.declare_parameter('survey_climb_step', 5.0)
        self.declare_parameter('investigate_orbit_radius', 4.0)
        self.declare_parameter('investigate_waypoint_count', 8)
        self.declare_parameter('return_altitude', 5.0)
        self.declare_parameter('auto_start', True)

        self.structure_center_north = self.get_parameter('structure_center_north').value
        self.structure_center_east = self.get_parameter('structure_center_east').value
        self.structure_height = self.get_parameter('structure_height').value
        self.survey_orbit_radius = self.get_parameter('survey_orbit_radius').value
        self.survey_climb_step = self.get_parameter('survey_climb_step').value
        self.investigate_orbit_radius = self.get_parameter('investigate_orbit_radius').value
        self.investigate_waypoint_count = self.get_parameter(
            'investigate_waypoint_count').value
        self.return_altitude = self.get_parameter('return_altitude').value
        self.auto_start = self.get_parameter('auto_start').value

        self.state = 'IDLE'
        self.defect_queue = collections.deque()
        self.waypoints_flown = 0
        self.defects_investigated = 0
        self.mission_start_time = None

        self.state_publisher = self.create_publisher(String, '/kestrel/mission_state', 10)
        self.create_subscription(
            DefectEvent, '/kestrel/defect_events', self.on_defect_event,
            10, callback_group=self.callback_group)

        self.takeoff_client = self.create_client(
            Takeoff, '/kestrel/cmd/takeoff', callback_group=self.callback_group)
        self.goto_client = self.create_client(
            GotoLocal, '/kestrel/cmd/goto', callback_group=self.callback_group)
        self.land_client = self.create_client(
            Trigger, '/kestrel/cmd/land', callback_group=self.callback_group)

        self.create_timer(1.0, self.publish_state)

        if self.auto_start:
            mission_thread = threading.Thread(target=self.run_mission, daemon=True)
            mission_thread.start()

    # Queue an incoming defect event for investigation
    def on_defect_event(self, defect_event_message):
        self.defect_queue.append(defect_event_message)

    # Publish the current state
    def publish_state(self):
        state_message = String()
        state_message.data = self.state
        self.state_publisher.publish(state_message)

    # Change state and publish immediately
    def set_state(self, new_state):
        self.state = new_state
        self.publish_state()

    # Block until the commander services exist, then run the mission
    def run_mission(self):
        self.mission_start_time = time.time()

        for client in (self.takeoff_client, self.goto_client, self.land_client):
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'waiting for service {client.srv_name}')

        self.set_state('TAKEOFF')
        start_altitude = max(MINIMUM_ALTITUDE, self.survey_climb_step)
        takeoff_request = Takeoff.Request()
        takeoff_request.altitude = start_altitude

        # ArduCopter prearm checks are not always settled the moment the
        # commander services exist, retry takeoff while they finish clearing
        takeoff_result = None
        takeoff_deadline = time.time() + TAKEOFF_READY_DEADLINE_SECONDS
        while time.time() < takeoff_deadline:
            takeoff_result = self.call_service_blocking(
                self.takeoff_client, takeoff_request, 200.0)
            if takeoff_result is not None and takeoff_result.success:
                break
            failure_message = takeoff_result.message if takeoff_result else 'no response'
            self.get_logger().warn(f'takeoff attempt failed: {failure_message}, retrying')
            time.sleep(TAKEOFF_RETRY_PAUSE_SECONDS)

        if takeoff_result is None or not takeoff_result.success:
            self.get_logger().error('takeoff failed, returning')
            self.return_and_land()
            return

        self.set_state('SURVEY')
        survey_path = build_survey_path(
            self.structure_center_north, self.structure_center_east,
            self.structure_height, self.survey_orbit_radius, self.survey_climb_step)

        # A straight line from home to the first waypoint crosses the
        # structure, climb above it and approach from over the top instead
        first_waypoint = survey_path[0]
        safe_altitude = self.structure_height + SAFE_TRANSIT_CLEARANCE_METERS
        climb_waypoint = Waypoint(north=0.0, east=0.0, altitude=safe_altitude, yaw_deg=0.0)
        approach_waypoint = Waypoint(
            north=first_waypoint.north, east=first_waypoint.east,
            altitude=safe_altitude, yaw_deg=first_waypoint.yaw_deg)
        for transit_waypoint in (climb_waypoint, approach_waypoint):
            if not self.fly_waypoint(transit_waypoint):
                self.get_logger().error('goto failed during transit to survey, returning')
                self.return_and_land()
                return

        # The loop index is not reset across INVESTIGATE and RESUME, so it
        # already remembers where the survey left off
        index = 0
        while index < len(survey_path):
            if not self.fly_waypoint(survey_path[index]):
                self.get_logger().error('goto failed during survey, returning')
                self.return_and_land()
                return
            self.waypoints_flown += 1
            index += 1

            if self.defect_queue:
                self.set_state('INVESTIGATE')
                while self.defect_queue:
                    defect_event = self.defect_queue.popleft()
                    defect_point = Waypoint(
                        north=defect_event.world_position.x,
                        east=defect_event.world_position.y,
                        altitude=defect_event.world_position.z,
                        yaw_deg=0.0)
                    orbit_path = build_orbit_path(
                        defect_point, self.investigate_orbit_radius,
                        self.investigate_waypoint_count)
                    for orbit_waypoint in orbit_path:
                        if not self.fly_waypoint(orbit_waypoint):
                            self.get_logger().error(
                                'goto failed during investigate, returning')
                            self.return_and_land()
                            return
                        self.waypoints_flown += 1
                    self.defects_investigated += 1
                self.set_state('RESUME')
                self.set_state('SURVEY')

        self.return_and_land()

    # Call goto for one waypoint and return success
    def fly_waypoint(self, waypoint):
        goto_request = GotoLocal.Request()
        goto_request.north = waypoint.north
        goto_request.east = waypoint.east
        goto_request.altitude = waypoint.altitude
        goto_request.yaw_deg = waypoint.yaw_deg
        result = self.call_service_blocking(self.goto_client, goto_request, 70.0)
        return result is not None and result.success

    # Fly home, land, log the mission summary, and set the landed state
    def return_and_land(self):
        self.set_state('RETURN')
        home_waypoint = Waypoint(north=0.0, east=0.0,
                                 altitude=self.return_altitude, yaw_deg=0.0)
        if not self.fly_waypoint(home_waypoint):
            self.get_logger().error('goto home failed, landing anyway')

        land_result = self.call_service_blocking(self.land_client, Trigger.Request(), 70.0)
        if land_result is None or not land_result.success:
            self.get_logger().error('land failed')

        self.set_state('LANDED')
        duration_seconds = time.time() - self.mission_start_time
        self.get_logger().info(
            f'mission complete: waypoints flown={self.waypoints_flown} '
            f'defects investigated={self.defects_investigated} '
            f'duration={duration_seconds:.1f}s')

    # Call a kestrel service and wait for the reply with a deadline
    def call_service_blocking(self, client, request, timeout_seconds):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'waiting for service {client.srv_name} to become available')

        future = client.call_async(request)

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
    mission_director = MissionDirector()
    executor = MultiThreadedExecutor()
    executor.add_node(mission_director)
    executor.spin()
    mission_director.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

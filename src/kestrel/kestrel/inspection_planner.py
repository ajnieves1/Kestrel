# Pure waypoint geometry for survey and closer look flights, no ROS in here
import math
from typing import NamedTuple

WAYPOINTS_PER_REVOLUTION = 12
MINIMUM_ALTITUDE = 3.0


# One flight target in the local frame with a compass heading
class Waypoint(NamedTuple):
    north: float
    east: float
    altitude: float
    yaw_deg: float


# Build a vertical helix of waypoints around a structure for survey flight
def build_survey_path(center_north, center_east, structure_height, orbit_radius, climb_step):
    start_altitude = max(MINIMUM_ALTITUDE, climb_step)
    altitude_step = climb_step / WAYPOINTS_PER_REVOLUTION

    waypoints = []
    index = 0
    while True:
        altitude = start_altitude + index * altitude_step
        if altitude > structure_height:
            break

        angle_deg = index * 360.0 / WAYPOINTS_PER_REVOLUTION
        north = center_north + orbit_radius * math.cos(math.radians(angle_deg))
        east = center_east + orbit_radius * math.sin(math.radians(angle_deg))
        yaw_deg = bearing_degrees(north, east, center_north, center_east)

        waypoints.append(Waypoint(north, east, altitude, yaw_deg))
        index += 1

    return waypoints


# Build a small circle of waypoints around one point for a closer look
def build_orbit_path(point, orbit_radius, waypoint_count):
    waypoints = []
    for index in range(waypoint_count):
        angle_deg = index * 360.0 / waypoint_count
        north = point.north + orbit_radius * math.cos(math.radians(angle_deg))
        east = point.east + orbit_radius * math.sin(math.radians(angle_deg))
        yaw_deg = bearing_degrees(north, east, point.north, point.east)

        waypoints.append(Waypoint(north, east, point.altitude, yaw_deg))

    return waypoints


# Compass bearing in degrees from one north east point to another
def bearing_degrees(from_north, from_east, to_north, to_east):
    north_delta = to_north - from_north
    east_delta = to_east - from_east
    bearing = math.degrees(math.atan2(east_delta, north_delta))
    return bearing % 360.0

# Unit tests for the pure waypoint geometry functions, no ROS here
import math

from kestrel.inspection_planner import (
    MINIMUM_ALTITUDE,
    WAYPOINTS_PER_REVOLUTION,
    Waypoint,
    bearing_degrees,
    build_orbit_path,
    build_survey_path,
)

TOLERANCE = 1e-6


# Every survey waypoint sits exactly orbit_radius from the center
def test_survey_path_radius():
    center_north, center_east = 15.0, 0.0
    orbit_radius = 8.0
    waypoints = build_survey_path(center_north, center_east, 22.0, orbit_radius, 5.0)
    for waypoint in waypoints:
        radius = math.sqrt(
            (waypoint.north - center_north) ** 2 + (waypoint.east - center_east) ** 2)
        assert abs(radius - orbit_radius) < TOLERANCE


# Consecutive survey waypoints climb by climb_step divided by waypoints per revolution
def test_survey_path_altitude_step():
    climb_step = 5.0
    waypoints = build_survey_path(15.0, 0.0, 22.0, 8.0, climb_step)
    expected_step = climb_step / WAYPOINTS_PER_REVOLUTION
    for previous, current in zip(waypoints, waypoints[1:]):
        assert abs((current.altitude - previous.altitude) - expected_step) < TOLERANCE


# The first survey waypoint starts at the minimum altitude or climb_step, whichever is larger
def test_survey_path_start_altitude():
    climb_step = 5.0
    waypoints = build_survey_path(15.0, 0.0, 22.0, 8.0, climb_step)
    assert abs(waypoints[0].altitude - max(MINIMUM_ALTITUDE, climb_step)) < TOLERANCE

    small_climb_step = 1.0
    small_climb_waypoints = build_survey_path(15.0, 0.0, 22.0, 8.0, small_climb_step)
    assert abs(
        small_climb_waypoints[0].altitude
        - max(MINIMUM_ALTITUDE, small_climb_step)) < TOLERANCE


# The survey path never generates a waypoint above the structure height
def test_survey_path_altitude_bound():
    structure_height = 22.0
    waypoints = build_survey_path(15.0, 0.0, structure_height, 8.0, 5.0)
    assert waypoints[-1].altitude <= structure_height + TOLERANCE


# Every survey waypoint yaw faces the center via the bearing formula
def test_survey_path_yaw_faces_center():
    center_north, center_east = 15.0, 0.0
    waypoints = build_survey_path(center_north, center_east, 22.0, 8.0, 5.0)
    for waypoint in waypoints:
        expected_yaw = bearing_degrees(
            waypoint.north, waypoint.east, center_north, center_east)
        assert abs(waypoint.yaw_deg - expected_yaw) < TOLERANCE


# The orbit path has the requested count, radius, and constant altitude
def test_orbit_path_shape():
    center = Waypoint(north=10.0, east=4.0, altitude=12.0, yaw_deg=0.0)
    orbit_radius = 4.0
    waypoint_count = 8
    waypoints = build_orbit_path(center, orbit_radius, waypoint_count)

    assert len(waypoints) == waypoint_count
    for waypoint in waypoints:
        radius = math.sqrt(
            (waypoint.north - center.north) ** 2 + (waypoint.east - center.east) ** 2)
        assert abs(radius - orbit_radius) < TOLERANCE
        assert abs(waypoint.altitude - center.altitude) < TOLERANCE
        expected_yaw = bearing_degrees(
            waypoint.north, waypoint.east, center.north, center.east)
        assert abs(waypoint.yaw_deg - expected_yaw) < TOLERANCE


# Bearing degrees matches the four cardinal compass directions
def test_bearing_degrees_cardinal_directions():
    assert abs(bearing_degrees(0.0, 0.0, 1.0, 0.0) - 0.0) < TOLERANCE
    assert abs(bearing_degrees(0.0, 0.0, 0.0, 1.0) - 90.0) < TOLERANCE
    assert abs(bearing_degrees(0.0, 0.0, -1.0, 0.0) - 180.0) < TOLERANCE
    assert abs(bearing_degrees(0.0, 0.0, 0.0, -1.0) - 270.0) < TOLERANCE

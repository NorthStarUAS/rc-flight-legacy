import math

from props import getNode
from auracore import wgs84

import comms.events

d2r = math.pi / 180.0
r2d = 180.0 / math.pi
sqrt_of_2 = math.sqrt(2.0)
gravity = 9.81                  # m/sec^2

circle_node = getNode("/task/circle/active", True)
pos_node = getNode("/position", True)
vel_node = getNode("/velocity", True)
orient_node = getNode("/orientation", True)
route_node = getNode("/task/route", True)
L1_node = getNode("/config/autopilot/L1_controller", True)
targets_node = getNode("/autopilot/targets", True)

def init():
    # sanity check, set some conservative values if none are
    # provided in the autopilot config
    if L1_node.getFloat("bank_limit_deg") < 0.1:
        L1_node.setFloat("bank_limit_deg", 25.0)
    if L1_node.getFloat("period") < 0.1:
        L1_node.setFloat("period", 25.0)

def update(dt):
    direction_str = circle_node.getString("direction")
    direction = 1.0
    if direction_str == "right":
        direction = -1.0
    elif direction_str == "left":
        direction = 1.0
    else:
        circle_node.setString("direction", "left")

    if pos_node.hasChild("longitude_deg") and \
         pos_node.hasChild("latitude_deg"):
        pos_lon = pos_node.getFloat("longitude_deg")
        pos_lat = pos_node.getFloat("latitude_deg")
    else:
        # no valid current position, bail out.
        return

    if circle_node.hasChild("longitude_deg") and \
       circle_node.hasChild("latitude_deg"):
        # we have a valid circle center
        center_lon = circle_node.getFloat("longitude_deg")
        center_lat = circle_node.getFloat("latitude_deg")
    else:
        # we have a valid position, but no valid circle center, use
        # current position.  (sanity fallback)
        circle_node.setFloat("longitude_deg", pos_lon)
        circle_node.setFloat("latitude_deg", pos_lat)
        circle_node.setFloat("radius_m", 100.0)
        circle_node.setString("direction", "left")
        center_lon = pos_lon
        center_lat = pos_lat

    # compute course and distance to center of target circle
    # fixme: should reverse this and direction sense to match 'land.py' and make more sense
    (course_deg, rev_deg, dist_m) = \
        wgs84.geo_inverse( pos_lat, pos_lon, center_lat, center_lon )

    # compute ideal ground course to be on the circle perimeter if at
    # ideal radius
    ideal_crs = course_deg + direction * 90
    if ideal_crs > 360.0: ideal_crs -= 360.0
    if ideal_crs < 0.0: ideal_crs += 360.0

    # (in)sanity check
    if circle_node.hasChild("radius_m"):
        radius_m = circle_node.getFloat("radius_m")
        if radius_m < 35: radius_m = 35
    else:
        radius_m = 100
        circle_node.setFloat("radius_m", 100.0)

    # compute a target ground course based on our actual radius
    # distance
    target_crs = ideal_crs
    if dist_m < radius_m:
        # inside circle, adjust target heading to expand our
        # circling radius
        offset_deg = direction * 90.0 * (1.0 - dist_m / radius_m)
        target_crs += offset_deg
    elif dist_m > radius_m:
        # outside circle, adjust target heading to tighten our
        # circling radius
        offset_dist = dist_m - radius_m
        if offset_dist > radius_m: offset_dist = radius_m
        offset_deg = direction * 90 * offset_dist / radius_m
        target_crs -= offset_deg
    if target_crs > 360.0: target_crs -= 360.0
    if target_crs < 0.0: target_crs += 360.0
    targets_node.setFloat( "groundtrack_deg", target_crs )

    # L1 'mathematical' response to error
    L1_period = L1_node.getFloat("period") # gain
    gs_mps = vel_node.getFloat("groundspeed_ms")
    omegaA = sqrt_of_2 * math.pi / L1_period
    VomegaA = gs_mps * omegaA
    course_error = orient_node.getFloat("groundtrack_deg") - target_crs
    # wrap to +/- 180
    if course_error < -180.0: course_error += 360.0
    if course_error >  180.0: course_error -= 360.0
    # clamp to +/-90
    if course_error < -90.0: course_error = -90.0
    if course_error > 90.0: course_error = 90.0
    targets_node.setFloat( "course_error_deg", course_error )

    # accel: is the lateral acceleration we need to compensate for
    # heading error
    accel = 2.0 * math.sin(course_error * d2r) * VomegaA

    # circling acceleration needed for our current distance from center
    if dist_m > 0.1:
        turn_accel = direction * gs_mps * gs_mps / dist_m
    else:
        turn_accel = 0.0

    # allow a crude fudge factor for non-straight airframes or imu
    # mounting errors.  This is essentially the bank angle that yields
    # zero turn rate
    bank_bias_deg = L1_node.getFloat("bank_bias_deg");

    # compute desired acceleration = acceleration required for course
    # correction + acceleration required to maintain turn at current
    # distance from center.
    total_accel = accel + turn_accel

    target_bank = -math.atan( total_accel / gravity )
    target_bank_deg = target_bank * r2d + bank_bias_deg

    bank_limit_deg = L1_node.getFloat("bank_limit_deg")
    if target_bank_deg < -bank_limit_deg + bank_bias_deg:
        target_bank_deg = -bank_limit_deg + bank_bias_deg
    if target_bank_deg > bank_limit_deg + bank_bias_deg:
        target_bank_deg = bank_limit_deg + bank_bias_deg

    targets_node.setFloat( "roll_deg", target_bank_deg )

    route_node.setFloat( "wp_dist_m", dist_m )
    if gs_mps > 0.1:
        route_node.setFloat( "wp_eta_sec", dist_m / gs_mps )
    else:
        route_node.setFloat( "wp_eta_sec", 0.0 )

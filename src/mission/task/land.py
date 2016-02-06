import math

from props import root, getNode

import comms.events
import mission.mission_mgr
from task import Task

d2r = math.pi / 180.0
r2d = 180.0 / math.pi
ft2m = 0.3048
m2ft = 1.0 / ft2m

class Land(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.task_node = getNode("/task", True)
        self.land_node = getNode("/task/land", True)
        self.route_node = getNode("/task/route", True)
        self.ap_node = getNode("/autopilot", True)
        self.pos_node = getNode("/position", True)
        self.vel_node = getNode("/velocity", True)
        self.orient_node = getNode("/orientation", True)
        self.engine_node = getNode("/controls/engine", True)
        self.imu_node = getNode("/sensors/imu", True)
        self.home_node = getNode("/task/home", True)
        self.targets_node = getNode("/autopilot/targets", True)

        # get task configuration parameters
        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")
        self.lateral_offset_m = config_node.getFloat("lateral_offset_m")
        self.glideslope_deg = config_node.getFloat("glideslope_deg")
        if self.glideslope_deg < 0.01:
            self.glideslope_deg = 6.0
        self.turn_radius_m = config_node.getFloat("turn_radius_m")
        if self.turn_radius_m < 1.0:
            self.turn_radisu_m = 75.0
        # self.turn_steps = config_node.getFloat("turn_steps") # depricated
        self.direction = config_node.getString("direction")
        if self.direction == "":
            self.direction = "left"
        self.extend_final_leg_m = config_node.getFloat("extend_final_leg_m")
        self.alt_bias_ft = config_node.getFloat("alt_bias_ft")
        self.approach_speed_kt = config_node.getFloat("approach_speed_kt")
        if self.approach_speed_kt < 0.1:
            self.approach_speed_kt = 25.0
        self.flare_pitch_deg = config_node.getFloat("flare_pitch_deg")
        self.flare_seconds = config_node.getFloat("flare_seconds")
        if self.flare_seconds < 0.1:
            self.flare_seconds = 5.0

        # copy to /task/land
        self.land_node.setFloat("lateral_offset_m", self.lateral_offset_m)
        self.land_node.setFloat("glideslope_deg", self.glideslope_deg)
        self.land_node.setFloat("turn_radius_m", self.turn_radius_m)
        # self.land_node.setInt("turn_steps", self.turn_steps) # depricated
        self.land_node.setString("direction", self.direction)
        self.land_node.setFloat("extend_final_leg_m", self.extend_final_leg_m)
        self.land_node.setFloat("altitude_bias_ft", self.alt_bias_ft)
        self.land_node.setFloat("approach_speed_kt", self.approach_speed_kt)
        self.land_node.setFloat("flare_pitch_deg", self.flare_pitch_deg)
        self.land_node.setFloat("flare_seconds", self.flare_seconds)

        self.last_lon = 0.0
        self.last_lat = 0.0
        self.last_az = 0.0

        self.side = -1.0
        self.approach_len_m = 0.0
        self.entry_agl_ft = 0.0
        self.flare = False
        self.flare_start_time = 0.0
        self.approach_throttle = 0.0
        self.approach_pitch = 0.0
        self.flare_pitch_range = 0.0

        self.saved_fcs_mode = ""
        self.saved_agl_ft = 0.0
        self.saved_speed_kt = 0.0

    def activate(self):
        if not self.active:
            # build the approach with the current property tree values
            self.build_approach()

            # compute the approach entry altitude from the designed
            # approach route length
            self.entry_agl_ft = self.approach_len_m * m2ft \
                * math.tan( self.land_node.getFloat("glideslope_deg") * d2r )
            # fixme
            # if display_on:
            #     printf("Approach distance m = %.1f, entry agl ft = %.1f\n",
            #            self.approach_len_m, self.entry_agl_ft)

            # Save existing state
            self.saved_fcs_mode = self.ap_node.getString("mode")
            self.saved_agl_ft = self.targets_node.getFloat("altitude_agl_ft")
            self.saved_speed_kt = self.targets_node.getFloat("airspeed_kt")

        self.ap_node.setString("mode", "basic+alt+speed")
        self.targets_node.setFloat("airspeed_kt",
                                   self.land_node.getFloat("approach_speed_kt"))

        # start at the beginning of the route (in case we inherit a
        # partially flown approach from earlier in the flight)
        # approach_mgr.restart() # FIXME
        self.flare = False

        self.active = True
        comms.events.log("mission", "land")
    
    def cart2polar(self, x, y):
        # fixme: if display_on:
	#    printf("approach %0f %0f\n", x, y);
        dist = math.sqrt(x*x + y*y)
        deg = math.atan2(x, y) * r2d
        return (dist, deg)

    def polar2cart(self, deg, dist):
        x = dist * math.sin(deg * d2r)
        y = dist * math.cos(deg * d2r)
        return (x, y)

    def update(self):
        if not self.active:
            return False

        # fixme: route_mgr_prof.start()
        # fixme: (not needed?) reposition_if_necessary()

        self.glideslope_rad = self.land_node.getFloat("glideslope_deg") * d2r
        self.extend_final_leg_m = self.land_node.getFloat("extend_final_leg_m")
        self.alt_bias_ft = self.land_node.getFloat("altitude_bias_ft")

        # compute glideslope/target elevation
        dist_m = self.route_node.getFloat("dist_remaining_m")
        alt_m = dist_m * math.tan(self.glideslope_rad)
        print "dist = %.1f alt = %.1f" % (dist_m, alt_m)
        
        # FIXME: this conditional action gets overwritten immediate after
        wpt_index = self.route_node.getInt("target_waypoint_idx")
        if wpt_index == 0:
            # fly entry altitude to first waypoint
            self.targets_node.setFloat("altitude_agl_ft",
                                       self.entry_agl_ft + self.alt_bias_ft)
        else:
            # fly glide slope after first waypoint
            self.targets_node.setFloat("altitude_agl_ft",
                                       alt_m * m2ft + self.alt_bias_ft )
        # fly glide slope after first waypoint
        self.targets_node.setFloat("altitude_agl_ft",
                                   alt_m * m2ft + self.alt_bias_ft )

        # compute error metrics
        alt_error_ft = self.pos_node.getFloat("altitude_agl_ft") - self.targets_node.getFloat("altitude_agl_ft")
        print "alt_error_ft = %.1f" % alt_error_ft
        # current_glideslope_deg = math.atan2(self.pos_node.getFloat("altitude_agl_m), dist_m) * r2d

        if wpt_index <= 1:
            # still tracking first or second waypoint of approach
            # route... (in case we start the approach over the first
            # waypoint and immediately find ourself tracking the
            # second one.)
            if alt_error_ft > 50.0:
                # more than 50' higher than target altitude, command a
                # circle descent to the approach entry altitude.

                # compute descending circle exit heading based on
                # approach direction (handedness)
                final_heading_deg = self.home_node.getFloat("azimuth_deg")
                exit_hdg = final_heading_deg
                dir = "left"
                if self.side < 0.0:
                    exit_hdg += 30.0
                    dir = "left"
                else:
                    exit_hdg -= 30.0
                    dir = "right"
                if exit_hdg < 0.0:
                    exit_hdg += 360.0
                if exit_hdg > 360.0:
                    exit_hdg -= 360.0

                self.radius_m = self.land_node.getFloat("turn_radius_m")
                x = self.radius_m * self.side
                y = -2.0*self.radius_m - self.extend_final_leg_m
                print "x,y:", (x, y)
                (offset_dist, offset_deg) = self.cart2polar(self.radius_m*self.side, -2.0*self.radius_m - self.extend_final_leg_m)
                print "dist,deg:", (offset_dist, offset_deg)
                circle_offset_deg = final_heading_deg + offset_deg
                if circle_offset_deg < 0.0:
                    circle_offset_deg += 360.0
                if circle_offset_deg > 360.0:
                    circle_offset_deg -= 360.0
                print "circle_offset_deg:", circle_offset_deg
                td = (self.home_node.getFloat("latitude_deg"),
                      self.home_node.getFloat("longitude_deg"))
                (cc_lat, cc_lon) = mission.greatcircle.project_course_distance( td, circle_offset_deg, offset_dist)
                
                print "requesting circle descent task"
                mission.mission_mgr.m.request_task_circle(cc_lon, cc_lat)
                mission.mission_mgr.m.request_task_circle_setup(self.radius_m,
                                                                dir)
                print "entry_agl=%.1f bias_ft=%.1f" % (self.entry_agl_ft, self.alt_bias_ft)
                mission.mission_mgr.m.request_task_circle_set_exit_conditions(
                        self.entry_agl_ft + self.alt_bias_ft, exit_hdg )

        # compute time to touchdown at current ground speed (assuming the
        # navigation system has lined us up properly
        ground_speed_ms = self.vel_node.getFloat("groundspeed_ms")
        if ground_speed_ms > 0.01:
            seconds_to_touchdown = dist_m / ground_speed_ms
        else:
            seconds_to_touchdown = 1000.0 # lots
            
        print "dist_m = %.1f gs = %.1f secs = %.1f" % \
            (dist_m, ground_speed_ms, seconds_to_touchdown)

        # approach_speed_kt = approach_speed_node.getFloat()
        self.flare_pitch_deg = self.land_node.getFloat("flare_pitch_deg")
        self.flare_seconds = self.land_node.getFloat("flare_seconds")

        if seconds_to_touchdown <= self.flare_seconds and not self.flare:
            # within x seconds of touchdown horizontally.  Note these
            # are padded numbers because we don't know the truth
            # exactly ... we could easily be closer or lower or
            # further or higher.  Our flare strategy is to smoothly
            # pull throttle to idle, while smoothly pitching to the
            # target flare pitch (as configured in the task
            # definition.)
            self.flare = True
            self.flare_start_time = self.imu_node.getFloat("timestamp")
            self.approach_throttle = self.engine_node.getFloat("throttle")
            self.approach_pitch = self.targets_node.getFloat("pitch_deg")
            self.flare_pitch_range = self.approach_pitch - self.flare_pitch_deg
            self.ap_node.setString("mode", "basic")

        if self.flare:
            if self.flare_seconds > 0.01:
                elapsed = self.imu_node.getFloat("timestamp") - self.flare_start_time
                percent = elapsed / self.flare_seconds
                if percent > 1.0:
                    percent = 1.0
                self.targets_node.setFloat("pitch_deg",
                                           self.approach_pitch
                                           - percent * self.flare_pitch_range)
                self.engine_node.setFloat("throttle",
                                          self.approach_throttle * (1.0 - percent))
                #printf("FLARE: elapsed=%.1f percent=%.2f speed=%.1f throttle=%.1f",
                #       elapsed, percent,
                #       approach_speed_kt - percent * self.flare_pitch_range,
                #       self.approach_throttle * (1.0 - percent))
            else:
                # printf("FLARE!!!!\n")
                self.targets_node.setFloat("pitch_deg", self.flare_pitch_deg)
                self.engine_node.setFloat("throttle", 0.0)

        # if ( display_on ) {
        #    printf("land dist = %.0f target alt = %.0f\n",
        #           dist_m, alt_m * SG_METER_TO_FEET + self.alt_bias_ft)

        # fixme: route_mgr_prof.stop()

    def is_complete(self):
        return False

        # Fixme: this would make more sense if we popped all the other tasks
        # off the stack first, the pushed land on the stack
        # ... otherwise we jump back to route or circle or whatever we
        # were doing before when this finishes.
        # Maybe request idle task? or pop all task and the push idle task?
        if self.task_node.getBool("is_airborne"):
            return False
        else:
            # FIXME: task is to land and we are on the ground, let's say we
            # are all done.
            return True;

    def close(self):
        # restore the previous state
        self.ap_node.setString("mode", self.saved_fcs_mode)
        self.targets_node.setFloat("airspeed_kt", self.saved_speed_kt)
        self.targets_node.setFloat("altitude_agl_ft", self.saved_agl_ft );
        self.active = False
        return True

    def build_approach(self):
        deg = 0.0
        dist = 0.0

        self.turn_radius_m = self.land_node.getFloat("turn_radius_m")
        self.extend_final_leg_m = self.land_node.getFloat("extend_final_leg_m")
        self.lateral_offset_m = self.land_node.getFloat("lateral_offset_m")
        self.side = -1.0
        dir = self.land_node.getString("direction")
        if dir == "left":
            self.side = -1.0
        elif dir == "right":
            self.side = 1.0

        # setup a descending circle aligned with the final turn to
        # final/base.  The actual approach route is simply two points.
        # When at the correct altitude and exit heading, drop the
        # circle and intercept the final approach.

        final_leg_m = 2.0 * self.turn_radius_m + self.extend_final_leg_m

        # fixme: if display_on:
        #    printf("side = %.1f\n", side)

        # create and request approach route
        route_request = "route"
        # start of final leg point
        (dist, deg) = self.cart2polar(self.lateral_offset_m * self.side,
                                      -final_leg_m)
        route_request += ",0,%.2f,%.2f,-" % (deg, dist)
        # touchdown point
        (dist, deg) = self.cart2polar(self.lateral_offset_m * self.side, 0.0)
        route_request += ",0,%.2f,%.2f,-" % (deg, dist)

        # set route request and route modes
        self.route_node.setString("route_request", route_request)
        self.route_node.setString("start_mode", "first_wpt")
        self.route_node.setString("follow_mode", "leader")
        self.route_node.setString("completion_mode", "extend_last_leg")
        
        # estimate approach length (final leg dist + 1/8 of the turning circle)
        self.approach_len_m = final_leg_m \
                              + self.turn_radius_m * 2.0*math.pi * 0.125

        # seed route dist_remaining_m value so it is not zero or lef
        # over from previous route.
        self.route_node.setFloat("dist_remaining_m", self.approach_len_m)

        # force a reposition on next update
        self.last_lon = 0.0
        self.last_lat = 0.0

        return True

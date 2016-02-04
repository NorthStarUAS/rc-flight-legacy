from props import root, getNode

import comms.events
from task import Task

d2r = math.pi / 180.0
ft2m = 0.3048
m2ft = 1.0 / ft2m

class Land(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.land_node = getNode("/task/land", True)
        self.task_node = getNode("/task", True)
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
        self.land_node.setString("direction", self.direction.c_str())
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
        if not is_active():
            # build the approach with the current property tree values
            self.build_approach()

            # compute the approach entry altitude from the designed
            # approach route length
            self.entry_agl_ft = approach_len_m * m2ft
                * math.tan( self.land_node.getFloat("glideslope_deg") * d2r )
            # fixme
            # if display_on:
            #     printf("Approach distance m = %.1f, entry agl ft = %.1f\n",
            #            approach_len_m, entry_agl_ft)

            # Save existing state
            self.saved_fcs_mode = self.ap_node.getString("mode")
            self.saved_agl_ft = targets_node.getFloat("altitude_agl_ft")
            self.saved_speed_kt = targets_node.getFloat("airspeed_kt")

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
        # fixme: approach_mgr.update()

        self.glideslope_rad = self.land_node.getFloat("glideslope_deg") * d2r
        self.extend_final_leg_m = self.land_node.getFloat("extend_final_leg_m")
        self.alt_bias_ft = self.land_node.getFloat("altitude_bias_ft")

        # compute glideslope/target elevation
        # FIXME: dist_m = approach_mgr.get_dist_remaining_m()
        alt_m = dist_m * math.tan(self.glideslope_rad)
        # FIXME: this conditional action gets overwritten immediate after
        if approach_mgr.get_waypoint_index() == 0:
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
        alt_error_ft = self.pos_node.getFloat("altitude_agl_ft") - self.targets_node.getFloat("target_agl_ft")
        # current_glideslope_deg = math.atan2(self.pos_node.getFloat("altitude_agl_m), dist_m) * r2d

        if approach_mgr.get_waypoint_index() <= 1:
            # still tracking first or second waypoint of approach
            # route... (in case we start the approach over the first
            # waypoint and immediately find ourself tracking the
            # second one.)
            if alt_error_ft > 50.0:
                # more than 50' higher than target altitude, command a
                # circle descent to the approach entry altitude.

                # compute descending circle exit heading based on
                # approach direction (handedness)
                exit_hdg = self.home_node.getFloat("azimuth_deg")
                dir = "left"
                if side < 0.0:
                    exit_hdg += 45.0
                    dir = "left"
                else:
                    exit_hdg -= 45.0
                    dir = "right"
                if exit_hdg < 0.0:
                    exit_hdg += 360.0
                if exit_hdg > 360.0:
                    exit_hdg -= 360.0

                radius_m = self.land_node.getFloat("turn_radius_m")
                offset_deg = 0.0
                offset_dist = 0.0
                (offset_dist, offset_deg) = cart2polar(radius_m*side, -2.0*radius_m - extend_final_leg_m)

                # printf("entry_agl=%.1f bias_ft=%.1f\n",
                #   entry_agl_ft, alt_bias_ft)
                mission_mgr.request_task_circle(
                        self.home_node.getFloat("longitude_deg"),
                        self.home_node.getFloat("latitude_deg"),
                        offset_deg, offset_dist )
                mission_mgr.request_task_circle_setup(
                        self.land_node.getFloat("turn_radius_m"),
                        dir )
                mission_mgr.request_task_circle_set_exit_conditions(
                        entry_agl_ft + alt_bias_ft, exit_hdg )

        # compute time to touchdown at current ground speed (assuming the
        # navigation system has lined us up properly
        ground_speed_ms = self.vel_node.getFloat("groundspeed_ms")
        if ground_speed_ms > 0.01:
            seconds_to_touchdown = dist_m / ground_speed_ms
        else:
            seconds_to_touchdown = 1000.0 # lots
            
        # printf("dist_m = %.1f gs = %.1f secs = %.1f\n",
        #    dist_m, ground_speed_ms, seconds_to_touchdown)

        # approach_speed_kt = approach_speed_node.getFloat()
        flare_pitch_deg = self.land_node.getFloat("flare_pitch_deg")
        flare_seconds = self.land_node.getFloat("flare_seconds")

        if seconds_to_touchdown <= flare_seconds and not self.flare:
            # within x seconds of touchdown horizontally.  Note these
            # are padded numbers because we don't know the truth
            # exactly ... we could easily be closer or lower or
            # further or higher.  Our flare strategy is to smoothly
            # pull throttle to idle, while smoothly pitching to the
            # target flare pitch (as configured in the task
            # definition.)
            self.flare = True
            flare_start_time = self.imu_node.getFloat("timestamp")
            approach_throttle = self.engine_node.getFloat("throttle")
            approach_pitch = self.targets_node.getFloat("pitch_deg")
            flare_pitch_range = approach_pitch - flare_pitch_deg
            self.ap_node.setString("mode", "basic")

        if self.flare:
            if flare_seconds > 0.01:
                elapsed = self.imu_node.getFloat("timestamp") - flare_start_time
                percent = elapsed / flare_seconds
                if percent > 1.0:
                    percent = 1.0
                self.targets_node.setFloat("pitch_deg",
                                           approach_pitch
                                           - percent * flare_pitch_range)
                self.engine_node.setFloat("throttle",
                                          approach_throttle * (1.0 - percent))
                #printf("FLARE: elapsed=%.1f percent=%.2f speed=%.1f throttle=%.1f",
                #       elapsed, percent,
                #       approach_speed_kt - percent * flare_pitch_range,
                #       approach_throttle * (1.0 - percent))
            else:
                # printf("FLARE!!!!\n")
                self.targets_node.setFloat("pitch_deg", flare_pitch_deg)
                self.engine_node.setFloat("throttle", 0.0)

        # if ( display_on ) {
        #    printf("land dist = %.0f target alt = %.0f\n",
        #           dist_m, alt_m * SG_METER_TO_FEET + alt_bias_ft)

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
        # fixme: approach_mgr.clear_standby()

        deg = 0.0
        dist = 0.0

        turn_radius_m = self.land_node.getFloat("turn_radius_m")
        extend_final_leg_m = self.land_node.getFloat("extend_final_leg_m")
        lateral_offset_m = self.land_node.getFloat("lateral_offset_m")
        side = -1.0
        dir = self.land_node.getString("direction")
        if dir == "left":
            side = -1.0
        elif dir == "right":
            side = 1.0

        # setup a descending circle aligned with the final turn to
        # final/base.  The actual approach route is simply two points.
        # When at the correct altitude and exit heading, drop the
        # circle and intercept the final approach.

        final_leg_m = 2.0 * self.turn_radius_m + self.extend_final_leg_m

        # fixme: if display_on:
        #    printf("side = %.1f\n", side)

        # start of final leg point
        (dist, deg) = self.cart2polar(self.lateral_offset_m * side,
                                      -self.final_leg_m)
        # fixme: approach_mgr.new_waypoint(dist, deg, 0.0, 0)

        # touchdown point
        (dist, deg) = self.cart2polar(self.lateral_offset_m * side, 0.0)
        # fixme: approach_mgr.new_waypoint( dist, deg, 0.0, 0 )

        # set route modes
        # fixme: approach_mgr.set_start_mode( FGRouteMgr::FIRST_WPT )
        # fixme: approach_mgr.set_follow_mode( FGRouteMgr::LEADER )
        # fixme: approach_mgr.set_completion_mode( FGRouteMgr::EXTEND_LAST_LEG )

        # estimate approach length (final leg dist + 1/8 of the turning circle)
        approach_len_m = final_leg_m + self.turn_radius_m * 2.0*math.pi * 0.125

        # make the new route 'active'
        # fixme: approach_mgr.swap()

        # force a reposition on next update
        self.last_lon = 0.0
        self.last_lat = 0.0

        return True

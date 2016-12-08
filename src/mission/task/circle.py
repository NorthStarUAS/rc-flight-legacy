import math

import sys
sys.path.append('/usr/local/lib')
import libnav_core

from props import root, getNode

import comms.events
from task import Task

d2r = math.pi / 180.0
r2d = 180.0 / math.pi
sqrt_of_2 = math.sqrt(2.0)
gravity = 9.81                  # m/sec^2

class Circle(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.pos_node = getNode("/position", True)
        self.vel_node = getNode("/velocity", True)
        self.orient_node = getNode("/orientation", True)
        self.circle_node = getNode("/task/circle", True)
        self.route_node = getNode("/task/route", True)
        self.L1_node = getNode("/config/autopilot/L1_controller", True)
        self.ap_node = getNode("/autopilot", True)
        self.targets_node = getNode("/autopilot/targets", True)

        self.coord_path = ""
        self.direction = "left"
        self.radius_m = 100.0
        self.target_agl_ft = 0.0
        self.target_speed_kt = 0.0
        self.exit_agl_ft = 0.0
        self.exit_heading_deg = 0.0
        
        self.saved_fcs_mode = ""
        self.saved_lon_deg = 0.0
        self.saved_lat_deg = 0.0
        self.saved_direction = ""
        self.saved_radius_m = 0.0
        self.saved_agl_ft = 0.0
        self.saved_speed_kt = 0.0

        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")
        self.coord_path = config_node.getString("coord_path")
        if config_node.hasChild("direction"):
            # only override the default if a value is given
            self.direction = config_node.getString("direction")
        if config_node.hasChild("radius_m"):
            # only override the default if a value is given
            self.radius_m = config_node.getFloat("radius_m")
        self.target_agl_ft = config_node.getFloat("altitude_agl_ft")
        self.target_speed_kt = config_node.getFloat("speed_kt")

        # sanity check, set some conservative values if none are
        # provided in the autopilot config
        if self.L1_node.getFloat("bank_limit_deg") < 0.1:
	    self.L1_node.setFloat("bank_limit_deg", 20.0)
        if self.L1_node.getFloat("period") < 0.1:
	    self.L1_node.setFloat("period", 25.0)

        # this needs to be done at the end if a coord_path is provided
        if self.coord_path != "":
            self.coord_node = getNode(self.coord_path, True)
        else:
            self.coord_node = None

    def activate(self):
        self.active = True

        # save existing state
        self.saved_fcs_mode = self.ap_node.getString("mode")
        self.saved_lon_deg = self.circle_node.getFloat("longitude_deg")
        self.saved_lat_deg = self.circle_node.getFloat("latitude_deg")

        self.saved_direction = self.circle_node.getString("direction")
        self.circle_node.setString("direction", self.direction)

        self.saved_radius_m = self.circle_node.getFloat("radius_m")
        self.circle_node.setFloat("radius_m", self.radius_m)

        # override target agl if requested by task
        if self.target_agl_ft > 0.0:
            self.saved_agl_ft = self.targets_node.getFloat("altitude_agl_ft")
            self.targets_node.setFloat("altitude_agl_ft", self.target_agl_ft)

        # override target speed if requested by task
        if self.target_speed_kt > 0.0:
            self.saved_speed_kt = self.targets_node.getFloat("airspeed_kt")
            self.targets_node.setFloat("airspeed_kt", self.target_speed_kt)

        # set fcs mode to basic+alt+speed
        self.ap_node.setString("mode", "basic+alt+speed")
        comms.events.log("mission", "circle")
    
    def update(self, dt):
        if not self.active:
            return False
        
        # update circle center if task specifies a source/coord path
        if self.coord_node and self.coord_node.hasChild("longitude_deg"):
            self.circle_node.setFloat("longitude_deg",
                                    self.coord_node.getFloat("longitude_deg"))
        if self.coord_node and self.coord_node.hasChild("latitude_deg"):
            self.circle_node.setFloat("latitude_deg",
                                    self.coord_node.getFloat("latitude_deg"))

        # circle_mgr update (code to fly the actual circle) is written
        # in C++ and located in src/control/circle_mgr.cxx  The
        # circle_mgr->update() routine is called from
        # src/control/control.cxx:update() whenever a circle hold task
        # is active.

        print "circle update"

        direction_str = self.circle_node.getString("direction")
        direction = 1.0
        if direction_str == "right":
            direction = -1.0

        center_lon = self.circle_node.getFloat("longitude_deg")
        center_lat = self.circle_node.getFloat("latitude_deg")

        # compute course and distance to center of target circle
        pos_lon = self.pos_node.getFloat("longitude_deg")
        pos_lat = self.pos_node.getFloat("latitude_deg")
        (course_deg, reverse_deg, dist_m) = \
            libnav_core.geo_inverse_wgs84( pos_lat, pos_lon,
                                           center_lat, center_lon )

        # compute ideal ground course to be on the circle perimeter if at
        # ideal radius
        ideal_crs = course_deg + direction * 90
        if ideal_crs > 360.0: ideal_crs -= 360.0
        if ideal_crs < 0.0: ideal_crs += 360.0

        # (in)sanity check
        radius_m = self.circle_node.getFloat("radius_m")
        if radius_m < 25.0: radius_m = 25.0

        # compute a target ground course based on our actual radius
        # distance
        target_crs = ideal_crs
        if dist_m < radius_m:
            # inside circle, adjust target heading to expand our
            # circling radius
            offset_deg = direction * 90.0 * (1.0 - dist_m / radius_m)
            target_crs += offset_deg
            if target_crs > 360.0: target_crs -= 360.0
            if target_crs < 0.0: target_crs += 360.0
        elif dist_m > radius_m:
            # outside circle, adjust target heading to tighten our
            # circling radius
            offset_dist = dist_m - radius_m
            if offset_dist > radius_m: offset_dist = radius_m
            offset_deg = direction * 90 * offset_dist / radius_m
            target_crs -= offset_deg
            if target_crs > 360.0: target_crs -= 360.0
            if target_crs < 0.0: target_crs += 360.0
        self.targets_node.setFloat( "groundtrack_deg", target_crs )
        # if ( display_on ) {
        # 	printf("rad=%.0f act=%.0f ideal crs=%.1f tgt crs=%.1f\n",
        # 	       radius_m, dist_m, ideal_crs, target_crs)
        # }

        # new L1 'mathematical' response to error

        L1_period = self.L1_node.getFloat("period") # gain
        gs_mps = self.vel_node.getFloat("groundspeed_ms")
        omegaA = sqrt_of_2 * math.pi / L1_period
        VomegaA = gs_mps * omegaA
        course_error = self.orient_node.getFloat("groundtrack_deg") - target_crs
        if course_error < -180.0: course_error += 360.0
        if course_error >  180.0: course_error -= 360.0
        self.targets_node.setFloat( "course_error_deg", course_error )

        # accel: is the lateral acceleration we need to compensate for
        # heading error
        accel = 2.0 * math.sin(course_error * d2r) * VomegaA

        # printf("debug: %.5f\t%.5f\t%.5f\n", orient_node.getFloat("groundtrack_deg"), target_crs, course_error)

        # printf("debug: %.5f\t%.5f\t%.5f\n", course_error, VomegaA, accel)

        # ideal_accel: the steady state lateral accel we would expect
        # when we are in the groove exactly on our target radius
        # double ideal_accel = direction * gs_mps * gs_mps / radius_m

        # circling acceleration needed for our current distance from center
        turn_accel = direction * gs_mps * gs_mps / dist_m

        # old way over turns when tracking inbound from a long distance
        # away
        # double total_accel = accel + ideal_accel

        # compute desired acceleration = acceleration required for course
        # correction + acceleration required to maintain turn at current
        # distance from center.
        total_accel = accel + turn_accel

        target_bank = -math.atan( total_accel / gravity )
        target_bank_deg = target_bank * r2d

        # printf("debug: %.6f\t%.6f\t%.5f\n", accel, turn_accel, target_bank_deg)

        bank_limit_deg = self.L1_node.getFloat("bank_limit_deg")
        if target_bank_deg < -bank_limit_deg: target_bank_deg = -bank_limit_deg
        if target_bank_deg > bank_limit_deg: target_bank_deg = bank_limit_deg
        # printf("   circle: tgt bank = %.0f  bank limit = %.0f\n",
        #	   target_bank_deg, bank_limit_deg)

        self.targets_node.setFloat( "roll_deg", target_bank_deg )

        # printf("circle: ideal ground crs = %.1f aircraft ground crs = %.1f\n",
        #	   course_deg, orient_node.getFloat("groundtrack_deg") )

        self.route_node.setFloat( "wp_dist_m", dist_m )
        if gs_mps > 0.1:
            self.route_node.setFloat( "wp_eta_sec", dist_m / gs_mps )
        else:
            self.route_node.setFloat( "wp_eta_sec", 0.0 )
        
    def is_complete(self):
        done = False
        # exit agl and exit heading specified
        if self.exit_agl_ft > 0.0:
            do_exit = True
            alt_agl_ft = self.pos_node.getFloat("altitude_agl_ft")
            if alt_agl_ft - self.exit_agl_ft > 25.0:
                # not low enough
                do_exit = False
            heading_deg = self.orient_node.getFloat("groundtrack_deg")
            if heading_deg < 0.0:
                heading_deg += 360.0
            hdg_error = heading_deg - self.exit_heading_deg
            if hdg_error < -180.0:
                hdg_error += 360.0
            if hdg_error > 180.0:
                hdg_error -= 360.0
            if abs(hdg_error) > 10.0:
                # not close enough
                do_exit = False
            done = do_exit
        return done
    
    def close(self):
        # restore the previous state
        self.ap_node.setString("mode", self.saved_fcs_mode)

        self.circle_node.setString("direction", self.saved_direction)
        self.circle_node.setFloat("radius_m", self.saved_radius_m)

        # restore target agl if overridden by task
        if self.target_agl_ft > 0.0:
            self.targets_node.setFloat("altitude_agl_ft", self.saved_agl_ft)

        # restore target speed if overridden by task
        if self.target_speed_kt > 0.0:
            self.targets_node.setFloat("airspeed_kt", saved_speed_kt)

        self.active = False
        return True

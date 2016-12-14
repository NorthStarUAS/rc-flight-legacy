import math

import sys
sys.path.append('/usr/local/lib')
import libnav_core

from props import root, getNode

import comms.events
from task import Task
import waypoint

d2r = math.pi / 180.0
r2d = 180.0 / math.pi
sqrt_of_2 = math.sqrt(2.0)
gravity = 9.81                  # m/sec^2

class Route(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.pos_node = getNode('/position', True)
        self.vel_node = getNode('/velocity', True)
        self.orient_node = getNode('/orientation', True)
        self.home_node = getNode('/task/home', True)
        self.route_node = getNode('/task/route', True)
        self.active_route_node = getNode('/task/route/active', True)
        self.L1_node = getNode('/config/autopilot/L1_controller', True)
        self.ap_node = getNode('/autopilot', True)
        self.targets_node = getNode('/autopilot/targets', True)
        self.gps_node = getNode('/sensors/gps', True)
        self.comms_node = getNode('/comms', True)

        self.alt_agl_ft = 0.0
        self.speed_kt = 30.0

        self.active_route = []        # actual routes
        self.standby_route = []
        self.current_wp = 0
        self.acquired = False
        
        self.last_lon = 0.0
        self.last_lat = 0.0
        self.last_az = 0.0
        self.wp_counter = 0
        self.dist_remaining = 0.0
        
        self.saved_fcs_mode = ''
        self.saved_agl_ft = 0.0
        self.saved_speed_kt = 0.0

        self.name = config_node.getString('name')
        self.nickname = config_node.getString('nickname')
        self.coord_path = config_node.getString('coord_path')
        self.alt_agl_ft = config_node.getFloat('altitude_agl_ft')
        self.speed_kt = config_node.getFloat('speed_kt')
        
        # sanity check, set some conservative values if none are
        # provided in the autopilot config
        if self.L1_node.getFloat('bank_limit_deg') < 0.1:
	    self.L1_node.setFloat('bank_limit_deg', 20.0)
        if self.L1_node.getFloat('period') < 0.1:
	    self.L1_node.setFloat('period', 25.0)
        if self.L1_node.getFloat('damping') < 0.1:
	    self.L1_node.setFloat('damping', 0.7)
            
        # defaults
        self.route_node.setString('follow_mode', 'leader');
        self.route_node.setString('start_mode', 'first_wpt');
        self.route_node.setString('completion_mode', 'loop');

        # load a route if included in config tree
        if self.build(config_node):
            self.swap()
        else:
            print 'Detected an internal inconsistency in the route'
	    print ' configuration.  See earlier errors for details.'
	    quit()
            
    def activate(self):
        self.active = True

        # save existing state
        self.saved_fcs_mode = self.ap_node.getString('mode')
        self.saved_agl_ft = self.targets_node.getFloat('altitude_agl_ft')
        self.saved_speed_kt = self.targets_node.getFloat('airspeed_kt')

        # set fcs mode to basic+alt+speed
        self.ap_node.setString('mode', 'basic+alt+speed')

        if self.alt_agl_ft > 0.1:
            self.targets_node.setFloat('altitude_agl_ft', self.alt_agl_ft)

        self.route_node.setString('follow_mode', 'leader');
        self.route_node.setString('start_mode', 'first_wpt');
        self.route_node.setString('completion_mode', 'loop');
        
        comms.events.log('mission', 'route')

    # build route from a property tree node
    def build(self, config_node):
        self.standby_route = []       # clear standby route
        for child_name in config_node.getChildren():
            if child_name == 'name':
                # ignore this for now
                pass                
            elif child_name[:3] == 'wpt':
                child = config_node.getChild(child_name)
                wp = waypoint.Waypoint()
                wp.build(child)
                self.standby_route.append(wp)
            elif child_name == 'enable':
                # we do nothing on this tag right now, fixme: remove
                # this tag from all routes?
                pass
            else:
                print 'Unknown top level section:', child_name
                return False
        print 'loaded %d waypoints' % len(self.standby_route)
        return True

    # build a route from a string request
    def build_str(self, request):
        tokens = request.split(',')
        if len(tokens) < 5:
            return False
        self.standby_route = []       # clear standby
        i = 1
        while i + 4 <= len(tokens):
            mode = int(tokens[i])
            wp = waypoint.Waypoint()
            if mode == 0:
                wp.mode = 'relative'
                wp.dist_m = float(tokens[i+1])
                wp.hdg_deg = float(tokens[i+2])
            else:
                wp.mode = 'absolute'
                wp.lon_deg = float(tokens[i+1])
                wp.lat_deg = float(tokens[i+2])
            self.standby_route.append(wp)
            i += 4;
        print 'loaded %d waypoints' % len(standby)
        return True

    # swap active and standby routes
    def swap(self):
        tmp = self.active_route
        self.active_route = self.standby_route
        self.standby_route = tmp
        self.current_wp = 0     # make sure we start at beginning

    def get_current_wp(self):
        if self.current_wp >= 0 and self.current_wp < len(self.active_route):
            return self.active_route[self.current_wp]
        else:
            return None
        
    def get_previous_wp(self):
        prev = self.current_wp - 1
        if prev < 0:
            prev = len(self.active_route) - 1
        if prev >= 0 and prev < len(self.active_route):
            return self.active_route[prev]
        else:
            return None

    def increment_current_wp(self):
        if self.current_wp < len(self.active_route) - 1:
            self.current_wp += 1
        else:
            self.current_wp = 0

    def dribble(self):
        # dribble active route into the active_node tree (one waypoint
        # per interation to keep the load consistent and light.)
        route_size = len(self.active_route)
        self.active_route_node.setInt('route_size', route_size)
        if route_size > 0:
            if self.wp_counter >= route_size:
                self.wp_counter = 0
            wp = self.active_route[self.wp_counter]
            wp_str = 'wpt[%d]' % self.wp_counter
            wp_node = self.active_route_node.getChild(wp_str, True)
            wp_node.setFloat("longitude_deg", wp.lon_deg)
            wp_node.setFloat("latitude_deg", wp.lat_deg)

            if self.wp_counter < route_size - 1:
                # compute leg course and distance
                next = self.active_route[self.wp_counter+1]
                (leg_course, reverse_course, leg_dist) = \
                    libnav_core.geo_inverse_wgs84( wp.lon_deg, wp.lat_deg,
                                                   next.lon_deg, next.lat_deg)
                wp.leg_dist_m = leg_dist
            self.wp_counter += 1

    def reposition(self):
        home_lon = self.home_node.getFloat("longitude_deg");
        home_lat = self.home_node.getFloat("latitude_deg");
        home_az = self.home_node.getFloat("azimuth_deg");

        if ( abs(home_lon - self.last_lon) > 0.000001 or
	     abs(home_lat - self.last_lat) > 0.000001 or
	     abs(home_az - self.last_az) > 0.001 ):
            for wp in self.active_route:
                if wp.mode == 'relative':
                    wp.update_relative_pos(home_lon, home_lat, home_az)
	    if self.comms_node.getBool('display_on'):
	        print "ROUTE pattern updated: %.6f %.6f (course = %.1f)" % \
                    (home_lon, home_lat, home_az)
	    self.last_lon = home_lon
	    self.last_lat = home_lat
	    self.last_az = home_az
            
    def get_remaining_distance_from_next_waypoint(self):
        result = 0
        for wp in self.active_route[self.current_wp:] :
            result += wp.leg_dist_m
        return result
    
    def update(self, dt):
        if not self.active:
            return False
        
        self.reposition()
        
        nav_course = 0.0
        nav_dist_m = 0.0

        request = self.route_node.getString('route_request')
        if len(request):
            result = ''
            if build_str(request):
                self.swap()
                self.reposition()
                result = 'success: ' + request
            else:
                result = 'failed: ' + request
            self.route_node.setString('request_result', result)
            self.route_node.setString('route_request', '')

        self.route_node.setInt('route_size', len(self.active_route))
        if len(self.active_route) > 0:
            if self.gps_node.getFloat("data_age") < 10.0:
                # track current waypoint of route (only!) if we have
                # recent gps data

                # route start up logic: if start_mode == first_wpt
                # then there is nothing to do, we simply continue to
                # track wpt 0 if that is the current waypoint.  If
                # start_mode == 'first_leg', then if we are tracking
                # wpt 0, increment it so we track the 2nd waypoint
                # along the first leg.  If only a 1 point route is
                # given along with first_leg startup behavior, then
                # don't do that again, force some sort of sane route
                # parameters instead!
                start_mode = self.route_node.getString('start_mode')
                if start_mode == 'first_leg' and self.current_wp == 0:
                    if len(self.active_route) > 1:
                        self.current_wp += 1
                    else:
                        self.route_node.setString('start_mode', 'first_wpt')
                        self.route_node.setString('follow_mode', 'direct')

                L1_period = self.L1_node.getFloat('period')
                L1_damping = self.L1_node.getFloat('damping')
                gs_mps = self.vel_node.getFloat('groundspeed_ms')

                prev = self.get_previous_wp()
                wp = self.get_current_wp()

                # compute direct-to course and distance
                pos_lon = self.pos_node.getFloat("longitude_deg")
                pos_lat = self.pos_node.getFloat("latitude_deg")
                (direct_course, reverse_course, direct_dist) = \
                    libnav_core.geo_inverse_wgs84( pos_lon, pos_lat,
                                                   wp.lon_deg, wp.lat_deg)
                print pos_lat, pos_lon, ":", wp.lat_deg, wp.lon_deg
                print ' course to:', direct_course, 'dist:', direct_dist
                
                # compute leg course and distance
                (leg_course, reverse_course, leg_dist) = \
                    libnav_core.geo_inverse_wgs84( prev.lon_deg, prev.lat_deg,
                                                   wp.lon_deg, wp.lat_deg)
                print ' leg course:', leg_course, 'dist:', leg_dist

                # difference between ideal (leg) course and direct course
                angle = leg_course - direct_course
                if angle < -180.0: angle += 360.0
                elif angle > 180.0: angle -= 360.0

                # compute cross-track error
                angle_rad = angle * d2r
                xtrack_m = math.sin(angle_rad) * direct_dist
                dist_m = math.cos(angle_rad) * direct_dist
                # print 'direct_dist = %.1f angle = %.1f dist_m = %.1f\n' % (direct_dist, angle, dist_m)
                self.route_node.setFloat( 'xtrack_dist_m', xtrack_m )
                self.route_node.setFloat( 'projected_dist_m', dist_m )

                # default distance for waypoint acquisition = direct
                # distance to the target waypoint.  This can be
                # overridden later by leg following and replaced with
                # distance remaining along the leg.
                nav_dist_m = direct_dist

                follow_mode = self.route_node.getString('follow_mode')
                completion_mode = self.route_node.getString('completion_mode')
                if follow_mode == 'direct':
                    # steer direct to
                    nav_course = direct_course
                elif follow_mode == 'xtrack_direct_hdg':
                    # cross track steering depricated.  See
                    # route_mgr.cxx in the historical archives for
                    # reference code.
                    pass
                elif follow_mode == 'xtrack_leg_hdg':
                    # cross track steering depricated.  See
                    # route_mgr.cxx in the historical archives for
                    # reference code.
                    pass
                elif follow_mode == 'leader':
                    L1_dist = (1.0 / math.pi) * L1_damping * L1_period * gs_mps
                    wangle = 0.0
                    if L1_dist < 0.01:
                        # ground speed <= 0.0 (problem?!?)
                        nav_course = direct_course
                    elif L1_dist <= abs(xtrack_m):
                        # beyond L1 distance, steer as directly toward
                        # leg as allowed
                        wangle = 0.0
                    else:
                        # steer towards imaginary point projected onto
                        # the route leg L1_distance ahead of us
                        wangle = math.acos(abs(xtrack_m) / L1_dist) * r2d
                    if wangle < 30.0: wangle = 30.0
                    if xtrack_m > 0.0:
                        nav_course = direct_course + angle - 90.0 + wangle
                    else:
                        nav_course = direct_course + angle + 90.0 - wangle
                    if self.acquired:
                        nav_dist_m = dist_m
                    else:
                        # direct to first waypoint until we've
                        # acquired this route
                        nav_course = direct_course
                        nav_dist_m = direct_dist

                    # printf('direct=%.1f angle=%.1f nav=%.1f L1=%.1f xtrack=%.1f wangle=%.1f nav_dist=%.1f\n', direct_course, angle, nav_course, L1_dist, xtrack_m, wangle, nav_dist_m)

                if nav_course < 0.0: nav_course += 360.0
                if nav_course > 360.0: nav_course -= 360.0

                self.targets_node.setFloat( 'groundtrack_deg', nav_course )

                # target bank angle computed here
                target_bank_deg = 0.0

                omegaA = sqrt_of_2 * math.pi / L1_period
                VomegaA = gs_mps * omegaA
                course_error = self.orient_node.getFloat('groundtrack_deg') \
                    - nav_course
                if course_error < -180.0: course_error += 360.0
                if course_error > 180.0: course_error -= 360.0
                self.targets_node.setFloat( 'course_error_deg', course_error )

                accel = 2.0 * math.sin(course_error * d2r) * VomegaA

                target_bank_deg = -math.atan( accel / gravity ) * r2d
                bank_limit_deg = self.L1_node.getFloat('bank_limit_deg')
                if target_bank_deg < -bank_limit_deg:
                    target_bank_deg = -bank_limit_deg
                if target_bank_deg > bank_limit_deg:
                    target_bank_deg = bank_limit_deg
                self.targets_node.setFloat( 'roll_deg', target_bank_deg )

                # estimate distance remaining to completion of route
                dist_remaining_m = nav_dist_m + \
                    self.get_remaining_distance_from_next_waypoint()
                self.route_node.setFloat('dist_remaining_m', dist_remaining_m)

                if self.comms_node.getBool('display_on'):
                    print 'next leg: %.1f  to end: %.1f  wpt=%d of %d' % (nav_dist_m, dist_remaining_m, self.current_wp, len(self.active_route))

                # logic to mark completion of leg and move to next leg.
                if completion_mode == 'loop':
                    if nav_dist_m < 50.0:
                        self.acquired = True
                        self.increment_current_wp()
                elif completion_mode == 'circle_last_wpt':
                    if nav_dist_m < 50.0:
                        self.acquired = True
                        if self.current_wp < len(self.active_route) - 1:
                            self.increment_current_wp()
                        else:
                            wp = self.get_current()
                            # FIXME: NEED TO GO TO CIRCLE MODE HERE SOME HOW!!!
                            #mission_mgr.request_task_circle(wp.get_target_lon(),
                            #   wp.get_target_lat(),
                            #   0.0, 0.0)
                elif completion_mode == 'extend_last_leg':
                    if nav_dist_m < 50.0:
                        self.acquired = True
                        if self.current_wp < len(self.active_route) - 1:
                            self.increment_current_wp()
                        else:
                            # follow the last leg forever
                            pass

                # publish current target waypoint
                self.route_node.setInt('target_waypoint_idx', self.current_wp)

                # if ( display_on ) {
                # printf('route dist = %0f\n', dist_remaining_m)
                # }
        else:
            pass
            # FIXME: we've been commanded to follow a route, but no
            # route has been defined.

            # We are in ill-defined territory, should we do some sort
            # of circle of our home position?

            # FIXME: need to go to circle mode somehow here!!!!
            # mission_mgr.request_task_circle()

        self.route_node.setFloat('wp_dist_m', direct_dist)

        gs_mps = self.vel_node.getFloat('groundspeed_ms')
        if gs_mps > 0.1:
            self.route_node.setFloat('wp_eta_sec', direct_dist / gs_mps)
        else:
            self.route_node.setFloat('wp_eta_sec', 0.0)

        # dribble active route into property tree
        self.dribble()

    def is_complete(self):
        return False
    
    def close(self):
        # restore the previous state
        self.ap_node.setString('mode', self.saved_fcs_mode)
        self.targets_node.setFloat('altitude_agl_ft', self.saved_agl_ft)
        self.targets_node.setFloat('airspeed_kt', self.saved_speed_kt)

        self.active = False
        return True

import math

from props import root, getNode

import comms.events
from task import Task

r2d = 180.0 / math.pi

class Launch(Task):
    def __init__(self, config_node):
        Task.__init__(self)

	self.ap_node = getNode("/autopilot", True)
	self.task_node = getNode("/task", True)
	self.pos_node = getNode("/position", True)
	self.vel_node = getNode("/velocity", True)
	self.orient_node = getNode("/orientation", True)
	self.ap_settings_node = getNode("/autopilot/settings", True)
	self.imu_node = getNode("/sensors/imu", True)
	self.flight_node = getNode("/controls/flight", True)
	self.engine_node = getNode("/controls/engine", True)

        self.complete_agl_ft = 150.0
        self.mission_agl_ft = 300.0
        self.target_speed_kt = 25.0
        self.roll_gain = 0.5
        self.roll_limit = 5.0
        self.rudder_enable = False
        self.rudder_gain = 1.0
        self.rudder_max = 1.0
        self.control_limit = 1.0

        self.last_ap_master = False
        self.last_imu_timestamp = 0.0
        self.relhdg = 0.0
        
        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")
        self.completion_agl_ft = config_node.getFloat("completion_agl_ft")
        self.mission_agl_ft = config_node.getFloat("mission_agl_ft")
        self.target_speed_kt = config_node.getFloat("speed_kt")
        self.roll_gain = config_node.getFloat("roll_gain")
        self.roll_limit = config_node.getFloat("roll_limit")
        self.rudder_enable = config_node.getBool("rudder_enable")
        self.rudder_gain = config_node.getFloat("rudder_gain")
        self.rudder_max = config_node.getFloat("rudder_max")

    def activate(self):
        self.active = True
        # start with roll control only, we fix elevator to neutral until
        # flight speeds come up and steer the rudder directly
        self.ap_node.setString("mode", "roll");
        self.ap_settings_node.setFloat("target_roll_deg", 0.0)
        self.ap_settings_node.setFloat("target_agl_ft", self.mission_agl_ft)
        self.ap_settings_node.setFloat("target_speed_kt", self.target_speed_kt)
    
    def update(self):
        if not self.active:
            return False
        
        # For wheeled take offs, track relative heading (initialized to
        # zero) when autopilot mode is engaged and steer that error to
        # zero with the rudder until flying/climbing

        if self.ap_node.getBool("master_switch"):
            imu_timestamp = self.imu_node.getFloat("timestamp")

            if not self.last_ap_master:
                # reset on entering AP mode
                self.relhdg = 0.0
                self.last_imu_timestamp = imu_timestamp
                self.control_limit = 1.0

            dt = imu_timestamp - self.last_imu_timestamp
            self.last_imu_timestamp = imu_timestamp

            throttle_time_sec = 2.0 # hard code for now (fixme: move to config)
            if dt > 0.0:
                # run up throttle over the specified interval
                throttle = self.engine_node.getFloat("throttle")
                throttle += dt / throttle_time_sec
                if throttle > 1.0:
                    throttle = 1.0
                self.engine_node.setFloat("throttle", throttle)

                # estimate short term heading
                self.relhdg += self.imu_node.getFloat("r_rad_sec") * r2d * dt

                # I am not clamping heading to +/- 180 here.  The
                # rational is that if we turn more than 180 beyond our
                # starting heading we are probably majorly screwed up,
                # but even so, let's unwind back the direction from
                # where we came rather than doing a hard-over reversal
                # of the rudder back to the other direction even if it
                # would be a shorter turning direction.  Less chance
                # of upsetting the apple cart with a massive control
                # input change.

                # if airborne, then slowly feather our max steer_limit
                # to zero over the period of 2 seconds.  This avoids a
                # hard rudder snap to zero if we are off heading, but
                # the assumption is once we are airborne we'll prefer
                # to keep our wings level and fly the current heading
                # rather than fight with our rudder and add drag to
                # get back on the heading.

                if self.task_node.getBool("is_airborne"):
                    feather_time = 5.0 # fixme: another config option
                    delta = ( 1.0 / feather_time ) * dt
                    self.control_limit -= delta
                    if self.control_limit < 0.0:
                        self.control_limit = 0.0

                if self.rudder_enable:
                    # simple proportional controller to steer against
                    # (estimated) heading error
                    rudder = self.relhdg * self.rudder_gain
                    steer_limit = self.rudder_max * self.control_limit
                    if rudder < -steer_limit:
                        rudder = -steer_limit
                    if rudder > steer_limit:
                        rudder = steer_limit
                    self.flight_node.setFloat("rudder", rudder)

                roll = -self.relhdg * self.roll_gain
                if roll < -self.roll_limit:
                    roll = -self.roll_limit
                if roll > self.roll_limit:
                    roll = self.roll_limit
                self.ap_settings_node.setFloat("target_roll_deg", roll)

                if self.vel_node.getFloat("airspeed_kt") > self.target_speed_kt:
                    # we've reached our flying/climbout airspeed,
                    # switch to pitch/elevator speed hold mode
                    self.ap_settings_node.setFloat("target_pitch_deg", self.orient_node.getFloat("pitch_deg"))
                    self.ap_node.setString("mode", "basic+alt+speed")

        self.last_ap_master = self.ap_node.getBool("master_switch")

    def is_complete(self):
        if self.pos_node.getFloat("altitude_agl_ft") >= self.complete_agl_ft:
            if self.rudder_enable:
                # just in case we get to the completion altitude
                # before we've feathered out the rudder input, let's
                # leave the rudder centered.
                self.flight_node.setFloat("rudder", 0.0)
            return True
        return False
    
    def close(self):
        self.active = False
        return True

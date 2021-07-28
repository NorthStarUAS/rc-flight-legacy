import math

from PropertyTree import PropertyNode

import comms.events
from mission.task.task import Task
from mission.task import fcsmode

r2d = 180.0 / math.pi

class Launch(Task):
    def __init__(self, config_node):
        Task.__init__(self)

        self.ap_node = PropertyNode("/autopilot")
        self.task_node = PropertyNode("/task")
        self.pos_node = PropertyNode("/position")
        self.vel_node = PropertyNode("/velocity")
        self.targets_node = PropertyNode("/autopilot/targets")
        self.imu_node = PropertyNode("/sensors/imu/0")
        self.flight_node = PropertyNode("/controls/flight")
        self.engine_node = PropertyNode("/controls/engine")

        self.complete_agl_ft = 150.0
        self.mission_agl_ft = 300.0
        self.target_speed_kt = 25.0
        self.roll_gain = 0.5
        self.roll_limit = 5.0
        self.rudder_enable = False
        self.rudder_gain = 1.0
        self.rudder_max = 1.0
        self.control_limit = 1.0
        self.flaps = 0.0

        self.last_ap_master = False
        self.relhdg = 0.0

        self.name = config_node.getString("name")
        self.completion_agl_ft = config_node.getDouble("completion_agl_ft")
        self.mission_agl_ft = config_node.getDouble("mission_agl_ft")
        self.target_speed_kt = config_node.getDouble("speed_kt")
        self.roll_gain = config_node.getDouble("roll_gain")
        self.roll_limit = config_node.getDouble("roll_limit")
        self.rudder_enable = config_node.getBool("rudder_enable")
        self.rudder_gain = config_node.getDouble("rudder_gain")
        self.rudder_max = config_node.getDouble("rudder_max")
        self.flaps = config_node.getDouble("flaps")
        self.target_pitch_deg = None
        if config_node.hasChild("target_pitch_deg"):
            self.target_pitch_deg = config_node.getDouble("target_pitch_deg")

    def activate(self):
        self.active = True
        # start with roll control only, we fix elevator to neutral until
        # flight speeds come up and steer the rudder directly
        if self.target_pitch_deg is None:
            fcsmode.set("roll");
        else:
            fcsmode.set("roll+pitch")
            self.targets_node.setDouble("pitch_deg", self.target_pitch_deg)
        self.targets_node.setDouble("roll_deg", 0.0)
        self.targets_node.setDouble("altitude_agl_ft", self.mission_agl_ft)
        self.targets_node.setDouble("airspeed_kt", self.target_speed_kt)

    def update(self, dt):
        if not self.active:
            return False

        throttle_time_sec = 2.0 # hard code for now (fixme: move to config)
        feather_time = 5.0      # fixme: make this a configurable option

        is_airborne = self.task_node.getBool("is_airborne")

        # For wheeled take offs, track relative heading (initialized to
        # zero) when autopilot mode is engaged and steer that error to
        # zero with the rudder until flying/climbing

        if self.ap_node.getBool("master_switch"):
            if not self.last_ap_master:
                # reset states when engaging AP mode
                self.relhdg = 0.0
                self.control_limit = 1.0
                self.flight_node.setDouble("flaps_setpoint", self.flaps)
                if not is_airborne:
                    # if engaging on the ground, start with zero throttle
                    self.engine_node.setDouble("throttle", 0.0)

            if not is_airborne:
                # run up throttle over the specified interval
                throttle = self.engine_node.getDouble("throttle")
                throttle += dt / throttle_time_sec
                if throttle > 1.0:
                    throttle = 1.0
                self.engine_node.setDouble("throttle", throttle)

            # estimate short term heading
            self.relhdg += self.imu_node.getDouble("r_rad_sec") * r2d * dt

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

            if is_airborne and self.control_limit > 0:
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
                self.flight_node.setDouble("rudder", rudder)

            roll = -self.relhdg * self.roll_gain
            if roll < -self.roll_limit:
                roll = -self.roll_limit
            if roll > self.roll_limit:
                roll = self.roll_limit
            self.targets_node.setDouble("roll_deg", roll)

            if is_airborne or \
               (self.vel_node.getDouble("airspeed_kt") > self.target_speed_kt):
                # we are flying or we've reached our flying/climbout
                # airspeed: switch to normal flight mode
                if fcsmode.get() != "basic+tecs":
                    fcsmode.set("basic+tecs")

        self.last_ap_master = self.ap_node.getBool("master_switch")

    def is_complete(self):
        if self.pos_node.getDouble("altitude_agl_ft") >= self.complete_agl_ft:
            # raise flaps
            self.flight_node.setDouble("flaps_setpoint", 0.0)

            # just in case we get to the completion altitude before
            # we've feathered out the rudder input, let's leave the
            # rudder centered.
            if self.rudder_enable:
                self.flight_node.setDouble("rudder", 0.0)

            return True
        else:
            return False

    def close(self):
        self.active = False
        return True

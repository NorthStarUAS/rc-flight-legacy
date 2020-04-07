# determine if aircraft if airborne or on the ground and time the
# airborne seconds

from props import getNode

import comms.events
from mission.task.task import Task

class IsAirborne(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.pos_node = getNode("/position", True)
        self.vel_node = getNode("/velocity", True)
        self.task_node = getNode("/task", True)
        self.status_node = getNode("/status", True)
        self.is_airborne = False
        self.off_alt_agl_ft = 0.0
        self.off_airspeed_kt = 0.0
        self.on_alt_agl_ft = 0.0
        self.on_airspeed_kt = 0.0
        self.name = config_node.getString("name")
        self.off_alt_agl_ft = config_node.getFloat("off_alt_agl_ft")
        self.off_airspeed_kt = config_node.getFloat("off_airspeed_kt")
        self.on_alt_agl_ft = config_node.getFloat("on_alt_agl_ft")
        self.on_airspeed_kt = config_node.getFloat("on_airspeed_kt")
        self.flight_accum = 0.0
        self.flight_start = 0.0

    def activate(self):
        self.active = True
        self.task_node.setBool("is_airborne", False)
        comms.events.log("mission", "on ground")

    def update(self, dt):
        if not self.active:
            return False

        if not self.is_airborne:
            # all conditions must be over the threshold in order to
            # become airborne
            cond = True
            if self.off_alt_agl_ft > 0.001 and \
               self.pos_node.getFloat("altitude_agl_ft") < self.off_alt_agl_ft:
                cond = False
            if self.off_airspeed_kt > 0.001 and \
               self.vel_node.getFloat("airspeed_kt") < self.off_airspeed_kt:
                cond = False
            if cond:
                self.is_airborne = True
                self.task_node.setBool("is_airborne", True)
                self.flight_start = self.status_node.getFloat('frame_time')
                comms.events.log("mission", "airborne")
        else:
            # if all conditions under their threshold, we are on the ground
            cond = True
            if self.on_alt_agl_ft > 0.001 and \
               self.pos_node.getFloat("altitude_agl_ft") > self.on_alt_agl_ft:
                cond = False
            if self.on_airspeed_kt > 0.001 and \
               self.vel_node.getFloat("airspeed_kt") > self.on_airspeed_kt:
                cond = False
            if cond:
                self.is_airborne = False
                self.task_node.setBool("is_airborne", False)
                # on ground, accumulate the elapsed airborne time
                elapsed = self.status_node.getFloat('frame_time') - self.flight_start
                self.flight_accum += elapsed
                comms.events.log("mission", "on ground")

        # compute total time aloft
        if self.is_airborne:
            flight_time = self.flight_accum + \
                          self.status_node.getFloat('frame_time') - self.flight_start
        else:
            flight_time = self.flight_accum
        self.task_node.setFloat('flight_timer', flight_time)

    def is_complete(self):
        return False

    def close(self):
        self.active = False
        return True

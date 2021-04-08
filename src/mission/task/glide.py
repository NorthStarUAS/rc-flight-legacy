import math

from props import getNode

import comms.events
from mission.task.task import Task
from mission.task import fcsmode
import mission.task.state

# this task flies a series of glide tests.  The config specifies max
# and min pitch angles and a pitch angle step.  Also a top and bottom
# altitude (agl).  The system will climb to the top altitude, set the
# starting pitch angle and hold that (throttle off) until the aircraft
# glides down to the bottom altitude.  It will then climb back up
# increment (or decrement) the pitch angle, and repeat through tthe
# whole pitch angle sweep range.
#
# navigation control is unaffected, so the aircraft could be put in a
# very large radius (200-300m) circle hold during the entire maneuver.

class GlideTest(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.targets_node = getNode("/autopilot/targets", True)
        self.config_node = config_node
        self.engine_node = getNode("/controls/engine", True)
        self.glide_node = getNode("/task/glide", True)
        self.pos_node = getNode("/position", True)
        self.name = config_node.getString("name")
        self.running = False
        self.last_enable = True # so we don't start out with a enable event

    def activate(self):
        self.active = True
    
    def start_experiment(self):
        # save current state
        mission.task.state.save(modes=True, circle=False, targets=False)

        # read config
        self.top_altitude = self.config_node.getFloat("top_agl_ft")
        if self.top_altitude < 200: self.top_altitude = 200
        self.bot_altitude = self.config_node.getFloat("bottom_agl_ft")
        if self.bot_altitude < 100: self.bot_altitude = 100
        self.pitch = self.config_node.getFloat("pitch_start_deg")
        if self.pitch > 10: self.pitch = 10
        if self.pitch < -20: self.pitch = -20
        self.pitch_end = self.config_node.getFloat("pitch_end_deg")
        if self.pitch_end > 10: self.pitch_end = 10
        if self.pitch_end < -20: self.pitch_end = -20
        self.pitch_incr = self.config_node.getFloat("pitch_increment")
        if abs(self.pitch_incr) <= 0.01: self.pitch_incr = 1
        # correct sign of increment value if needed
        if self.pitch > self.pitch_end and self.pitch_incr > 0:
            self.pitch_incr = -self.pitch_incr
        if self.pitch < self.pitch_end and self.pitch_incr < 0:
            self.pitch_incr = -self.pitch_incr

        self.glide_node.setFloat("pitch", self.pitch)

        # configure initial climb to top altitude
        self.targets_node.setFloat("altitude_agl_ft", self.top_altitude)
        fcsmode.set("basic+tecs")

        self.running = True
        
        comms.events.log("glide", "task started")

    def end_experiment(self, abort=False):
        # restore previous state
        mission.task.state.restore()
        if abort:
            # ended early by operator
            comms.events.log("glide", "aborted by operator")
        else:
            # experiment ran to completion
            comms.events.log("glide sequence", "completed")
        self.running = False
        self.glide_node.setBool("enable", False)

    def update(self, dt):
        if not self.active:
            return False

        # test for start enable
        enable = self.glide_node.getBool("enable")
        if enable and not self.last_enable:
            self.start_experiment()

        # test if enable switched off while running experiment
        if self.running and not enable and self.last_enable:
            self.end_experiment(abort=True)

        if self.running:
            # test for experiment finished
            if self.pitch_incr < 0 and self.pitch < self.pitch_end - 0.01:
                self.end_experiment()
            if self.pitch_incr > 0 and self.pitch > self.pitch_end + 0.01:
                self.end_experiment()

            # monitor for state transitions
            alt = self.pos_node.getFloat("altitude_agl_ft")
            if alt < self.bot_altitude + 10:
                if fcsmode.get() != "basic+tecs":
                    fcsmode.set("basic+tecs")
                    self.pitch += self.pitch_incr
                    comms.events.log("glide", "start climb")
            if alt > self.top_altitude - 10:
                if fcsmode.get() != "basic" :
                    fcsmode.set("basic")
                    self.glide_node.setFloat("pitch", self.pitch)
                    self.targets_node.setFloat("pitch_deg", self.pitch)
                    self.engine_node.setFloat("throttle", 0.0)
                    comms.events.log("glide", "start decent pitch = " + str(self.pitch))

        self.glide_node.setBool("running", self.running)
        self.last_enable = enable
        
    def is_complete(self):
        # this is intended to be a global task such that is_complete()
        # will never actually be called (the individual experiements
        # are sequenced and timed within this task.)
        return False
    
    def close(self):
        self.active = False
        return True

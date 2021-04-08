import math

from props import getNode

import comms.events
from mission.task.task import Task
from mission.task import fcsmode
import mission.task.state

# this task generates various excitation signals (chirps, doublets,
# multi-sines) which can then be applied to various control inputs.
# The expectation is that these excitations will be overlaid on top of
# the autopilot outputs which will lead to some 'flighting' but the
# aircraft will stay on condition better and there still should be
# enough excitation for all the post flight analysis.

# 'self.index' implements an experiment indexing scheme.  This enables
# the operator to queue an arbitarily long sequence of different
# excitations that can be run in sequence automatically.  Anticipating
# some external source could set the experiment number, we need to
# validate the index before using it.  Also anticipating the external
# source could be a transmitter toggle switch, the index will wrap
# around when it exceeds the allowable values.

class GlideTest(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.targets_node = getNode("/autopilot/targets", True)
        self.config_node = config_node
        self.engine_node = getNode("/controls/engine", True)
        self.glide_node = getNode("/task/glide", True)
        self.pos_node = getNode("/position", True)
        self.name = config_node.getString("name")
        self.start_time = 0.0
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

        self.glide_node.setString("state", "")
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
            # only log an event if the abort happens when the
            # glide is running
            comms.events.log("glide", "aborted by operator")
        else:
            # experiment ran to completion, increment experiment
            # index.
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

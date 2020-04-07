import math

from props import getNode

import comms.events
from mission.task.task import Task

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

class Excite(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.imu_node = getNode("/sensors/imu", True)
        self.config_node = config_node
        self.excite_node = getNode("/task/excite", True)
        self.name = config_node.getString("name")
        self.index = 0
        self.start_time = 0.0
        self.type = ''
        self.running = False
        self.last_trigger = True # so we don't start out with a trigger event

    def activate(self):
        self.active = True
    
    def start_experiment(self):
        max = self.config_node.getLen('experiment')
        print('number of experiments:', max)
        if self.index < 0:
            self.index = max - 1
        if self.index >= max:
            self.index = 0

        self.exp_node = self.config_node.getChild("experiment[%d]" % self.index)

        self.type = self.exp_node.getString('type')
        event_log = self.type

        self.target = []
        self.channels = self.exp_node.getLen("target")
        self.excite_node.setInt("channels", self.channels)
        self.excite_node.setLen("signal", self.channels, 0.0)
        self.excite_node.setLen("target", self.channels, "")
        for i in range(self.channels):
            v = self.exp_node.getStringEnum("target", i)
            self.target.append(v)
            self.excite_node.setStringEnum("target", i, v)
        event_log += ' ' + str(self.target)

        # For chirp and oms duration is the time of the total
        # excitation.  For pulse and doublet duration is the length of
        # one unit.
        if self.type == "oms" or self.type == "chirp":
            self.dur_sec = self.exp_node.getFloat("duration_sec")
            if self.dur_sec < 1:  self.dur_sec = 1
            if self.dur_sec > 100: self.dur_sec = 100
            event_log += ' ' + str(self.dur_sec)
        else:
            unit_sec = self.exp_node.getFloat("duration_sec")
            if self.type == "pulse":
                self.dur_sec = unit_sec
            elif self.type == "doublet":
                self.dur_sec = 2 * unit_sec
            elif self.type == "doublet121":
                self.dur_sec = 4 * unit_sec
            elif self.type == "doublet3211":
                self.dur_sec = 7 * unit_sec
            event_log += ' ' + str(unit_sec)

        self.amplitude = []
        n = self.exp_node.getLen("amplitude")
        for i in range(n):
            v = self.exp_node.getFloatEnum("amplitude", i)
            self.amplitude.append(v)
        event_log += ' ' + 'ampl: ' + str(self.amplitude)
        
        if self.type == "chirp":
            # rad/sec is hz*2*pi
            self.freq_start = []
            n = self.exp_node.getLen("freq_start_rad_sec")
            for i in range(n):
                v = self.exp_node.getFloatEnum("freq_start_rad_sec", i)
                if v < 0.1:   v = 0.1
                if v > 100.0: v = 100.0
                self.freq_start.append(v)
            event_log += ' ' + str(self.freq_start)
            
            self.freq_end = []
            n = self.exp_node.getLen("freq_end_rad_sec")
            for i in range(n):
                v = self.exp_node.getFloatEnum("freq_end_rad_sec", i)
                if v < 0.1:   v = 0.1
                if v > 100.0: v = 100.0
                self.freq_end.append(v)
            event_log += ' ' + str(self.freq_end)

            if len(self.freq_start) == len(self.freq_end):
                n = len(self.freq_start)
                self.k = []
                for i in range(n):
                    v = (self.freq_end[i] - self.freq_start[i]) / (2 * self.dur_sec)
                    self.k.append(v)
        elif self.type == "oms":
            self.freq_rps = []
            n = self.exp_node.getLen("freq_rps")
            for i in range(n):
                v = self.exp_node.getFloatEnum("freq_rps", i)
                self.freq_rps.append(v)
            self.phase_rad = []
            n = self.exp_node.getLen("phase_rad")
            for i in range(n):
                v = self.exp_node.getFloatEnum("phase_rad", i)
                self.phase_rad.append(v)
            
            n_coeffs = n / self.channels
            self.scale = math.sqrt(1.0 / n_coeffs)
                
        self.start_time = self.imu_node.getFloat("timestamp")
        self.running = True

        comms.events.log("excite", event_log)
        if self.type == "oms":
            # log oms arrays separately because they could get big
            comms.events.log("excite", 'freq: ' + str(self.freq_rps))
            comms.events.log("excite", 'phase: ' + str(self.phase_rad))

    def update_experiment(self, t):
        progress = t / self.dur_sec
        self.excite_node.setFloat("progress", progress)
        for i in range(self.channels):
            signal = 0.0
            if self.type == 'pulse':
                signal = self.amplitude[i]
            elif self.type == 'doublet':
                if progress >= 0.5:
                    signal = -self.amplitude[i]
                else:
                    signal = self.amplitude[i]
            elif self.type == 'doublet121':
                if progress >= 0.75:
                    signal = self.amplitude[i]
                elif progress >= 0.25:
                    signal = -self.amplitude[i]
                else:
                    signal = self.amplitude[i]
            elif self.type == 'doublet3211':
                if progress >= (6.0 / 7.0):
                    signal = -self.amplitude[i]
                elif progress >= (5.0 / 7.0):
                    signal = self.amplitude[i]
                elif progress >= (3.0 / 7.0):
                    signal = -self.amplitude[i]
                else:
                    signal = self.amplitude[i]
            elif self.type == 'chirp':
                signal = self.amplitude[i] * math.sin(self.freq_start[i]*t + self.k[i]*t*t)
            elif self.type == 'oms':
                # Optimal MultiSine
                n = len(self.freq_rps) / self.channels
                signal = 0.0
                for j in range(n*i, n*i+n):
                    signal += self.scale * self.amplitude[j] \
                              * math.cos(self.freq_rps[j] * t + self.phase_rad[j])
            self.excite_node.setFloatEnum("signal", i, signal)

    def end_experiment(self, abort=False):
        if abort:
            # only log an event if the abort happens when the
            # excitation is running
            self.excite_node.setFloat("progress", 0.0)
            comms.events.log("excite", "aborted by operator")
        else:
            # experiment ran to completion, increment experiment
            # index.
            self.excite_node.setFloat("progress", 1.0)
            self.index += 1
            comms.events.log("excite", "completed")
        n = len(self.target)
        for i in range(n):
            self.excite_node.setFloatEnum("signal", i, 0.0)
        self.running = False

    def update(self, dt):
        if not self.active:
            return False

        cur_time = self.imu_node.getFloat("timestamp")

        # test for start trigger
        trigger = self.excite_node.getBool("trigger")
        if trigger and not self.last_trigger:
            self.start_experiment()

        # test if trigger switched off while running experiment
        if self.running and not trigger and self.last_trigger:
            self.end_experiment(abort=True)

        # test if experiment has run to completion
        if self.running and cur_time > self.start_time + self.dur_sec:
            self.end_experiment()

        if self.running:
            t = cur_time - self.start_time
            self.update_experiment(t)

        self.excite_node.setBool("running", self.running)
        self.last_trigger = trigger
        
    def is_complete(self):
        # this is intended to be a global task such that is_complete()
        # will never actually be called (the individual experiements
        # are sequenced and timed within this task.)
        return False
    
    def close(self):
        self.active = False
        return True

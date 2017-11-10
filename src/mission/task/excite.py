import math

from props import getNode

import comms.events
from task import Task

# this task generates various excitation signals (chirps, doublets,
# multi-sines) which can then be applied to various control inputs.
# The expectation is that these excitations will be overlaid on top of
# the autopilot outputs which will lead to some 'flighting' but the
# aircraft will stay on condition better and there still should be
# enough excitation for all the post flight analysis.

class Excite(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.imu_node = getNode("/sensors/imu", True)
        self.config_node = config_node
        self.excite_node = getNode("/task/excite", True)
        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")
        self.index = 0
        self.start_time = 0.0
        self.type = ''
        self.running = False
        self.last_trigger = True # so we don't start out with a trigger event

    def activate(self):
        self.active = True
    
    def start_experiment(self):
        max = self.config_node.getLen('experiment')
        print 'number of experiments:', max
        if self.index >= max:
            self.index = 0

        self.exp_node = self.config_node.getChild("experiment[%d]" % self.index)

        event_log = ''
        
        self.type = self.exp_node.getString('type')
        self.target = self.exp_node.getString("target")
        self.excite_node.setString("target", self.target)
        event_log += self.type + ' ' + self.target

        self.dur_sec = self.exp_node.getFloat("duration_sec")
        if self.dur_sec < 1:  self.dur_sec = 1
        if self.dur_sec > 100: self.dur_sec = 100
        event_log += ' ' + str(self.dur_sec)

        if self.type != "oms":
            self.amplitude = self.exp_node.getFloat("amplitude")
            if self.amplitude < 0.001:  self.amplitude = 0.001
            if self.amplitude > 1: self.amplitude = 1
            event_log += ' ' + str(self.amplitude)
        
        if self.type == "chirp":
            # rad/sec is hz*2*pi
            self.freq_start = self.exp_node.getFloat("freq_start_rad_sec")
            if self.freq_start < 0.1:   self.freq_start = 0.1
            if self.freq_start > 100.0: self.freq_start = 100.0
            event_log += ' ' + str(self.freq_start)
            
            self.freq_end = self.exp_node.getFloat("freq_end_rad_sec")
            if self.freq_end < 0.1:  self.freq_end = 0.1
            if self.freq_end > 100.0: self.freq_end = 100.0
            event_log += ' ' + str(self.freq_end)
            
            self.k = (self.freq_end - self.freq_start) / (2 * self.dur_sec)
        elif self.type == "oms":
            self.freq_rps = []
            n = self.exp_node.getLen("freq_rps")
            if n > 0:
                for i in range(n):
                    v = self.exp_node.getFloatEnum("freq_rps", i)
                    self.freq_rps.append(v)
            self.phase_rad = []
            n = self.exp_node.getLen("phase_rad")
            if n > 0:
                for i in range(n):
                    v = self.exp_node.getFloatEnum("phase_rad", i)
                    self.phase_rad.append(v)
            self.amplitude = []
            n = self.exp_node.getLen("amplitude")
            if n > 0:
                for i in range(n):
                    v = self.exp_node.getFloatEnum("amplitude", i)
                    self.amplitude.append(v)
            self.scale = math.sqrt(1.0 / n)

            event_log += ' ' + str(self.freq_rps)
            event_log += ' ' + str(self.phase_rad)
            event_log += ' ' + str(self.amplitude)
                
        self.start_time = self.imu_node.getFloat("timestamp")
        self.running = True

        comms.events.log("excite", event_log)

    def update_experiment(self, t):
        progress = t / self.dur_sec
        signal = 0.0
        if self.type == 'chirp':
            signal = self.amplitude * math.sin(self.freq_start*t + self.k*t*t)
        elif self.type == 'doublet':
            if progress >= 0.5:
                signal = -self.amplitude
            else:
                signal = self.amplitude
        elif self.type == 'doublet121':
            if progress >= 0.75:
                signal = self.amplitude
            elif progress >= 0.25:
                signal = -self.amplitude
            else:
                signal = self.amplitude
        elif self.type == 'doublet3211':
            if progress >= (6.0 / 7.0):
                signal = -self.amplitude
            elif progress >= (5.0 / 7.0):
                signal = self.amplitude
            elif progress >= (3.0 / 7.0):
                signal = -self.amplitude
            else:
                signal = self.amplitude
        elif self.type == 'oms':
            # Optimal MultiSine
	    n = len(self.freq_rps)
	    signal = 0.0
	    for i in range(n):
		signal += self.scale * self.amplitude[i] \
                          * math.cos(self.freq_rps[i] * t + self.phase_rad[i])
        elif self.type == 'pulse':
            signal = self.amplitude

        self.excite_node.setFloat("signal", signal)
        self.excite_node.setFloat("progress", progress)

    def end_experiment(self, abort=False):
        if abort:
            # only log an event if the abort happens when the
            # excitation is running
            comms.events.log("excite", "aborted by operator")
        else:
            comms.events.log("excite", "completed")
        self.excite_node.setFloat("signal", 0.0)
        self.excite_node.setFloat("progress", 0.0)
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
        # will never actually be called
        return False
    
    def close(self):
        self.active = False
        return True

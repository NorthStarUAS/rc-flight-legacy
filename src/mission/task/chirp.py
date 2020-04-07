import math

from props import getNode

import comms.events
from mission.task.task import Task

class Chirp(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.imu_node = getNode("/sensors/imu", True)
        self.chirp_node = getNode("/task/chirp", True)
        self.signal_node = getNode("/controls/signal", True)
        self.name = config_node.getString("name")
        self.start_time = 0.0
        self.k = 0.0
        # rad/sec is hz*2*pi
        if config_node.hasChild("freq_start_rad_sec"):
            self.freq_start = float(config_node.getString("freq_start_rad_sec"))
            if self.freq_start < 0.1:   self.freq_start = 0.1
            if self.freq_start > 100.0: self.freq_start = 100.0
        else:
            self.freq_start = 6.2830
        self.chirp_node.setFloat("freq_start_rad_sec", self.freq_start)
        if config_node.hasChild("freq_end_rad_sec"):
            self.freq_end = float(config_node.getString("freq_end_rad_sec"))
            if self.freq_end < 0.1:  self.freq_end = 0.1
            if self.freq_end > 100.0: self.freq_end = 100.0
        else:
            self.freq_end = 62.83
        self.chirp_node.setFloat("freq_end_rad_sec", self.freq_end)
        if config_node.hasChild("duration_sec"):
            self.dur_sec = float(config_node.getString("duration_sec"))
            if self.dur_sec < 1:  self.dur_sec = 1
            if self.dur_sec > 100: self.dur_sec = 100
        else:
            self.dur_sec = 20            
        self.chirp_node.setFloat("duration_sec", self.dur_sec)
        if config_node.hasChild("amplitude"):
            self.amplitude = float(config_node.getString("amplitude"))
            if self.amplitude < 0.001:  self.amplitude = 0.001
            if self.amplitude > 1: self.amplitude = 1
        else:
            self.amplitude = 0.1
        self.chirp_node.setFloat("amplitude", self.amplitude)
        if config_node.hasChild("inject"):
            self.inject = config_node.getString("inject")
        else:
            self.inject = "aileron"
        self.signal_node.setString("inject", self.inject)
        self.last_trigger = True # so we don't start out with a trigger event
        self.running = False
        
    def activate(self):
        self.active = True
    
    def update(self, dt):
        if not self.active:
            return False

        # test for start trigger
        trigger = self.chirp_node.getBool("trigger")
        if trigger and not self.last_trigger:
            self.freq_start = self.chirp_node.getFloat("freq_start_rad_sec")
            self.freq_end = self.chirp_node.getFloat("freq_end_rad_sec")
            self.dur_sec = self.chirp_node.getFloat("duration_sec")
            self.amplitude = self.chirp_node.getFloat("amplitude")
            self.start_time = self.imu_node.getFloat("timestamp")
            self.k = (self.freq_end - self.freq_start) / (2 * self.dur_sec)
            self.running = True
            comms.events.log("chirp", self.signal_node.getString("inject"))
            comms.events.log("chirp", "start freq %.2f rad/sec" % self.freq_start)
            comms.events.log("chirp", "amplitude %.2f" % self.amplitude)

        if not trigger and self.last_trigger:
            if self.running:
                # only log an event if the abort happens when the
                # chirp is running
                comms.events.log("chirp", "aborted by operator")
            self.running = False
            
        cur_time = self.imu_node.getFloat("timestamp")
        if cur_time > self.start_time + self.dur_sec and self.running:
            comms.events.log("chirp", "end freq %.2f rad/sec" % self.freq_end)
            self.running = False

        if self.running:
            t = cur_time - self.start_time
            chirp = self.amplitude * math.sin(self.freq_start*t + self.k*t*t)
            self.signal_node.setFloat("value", chirp)
            self.signal_node.setFloat("progress", t)
        else:
            self.signal_node.setFloat("value", 0.0)
            self.signal_node.setFloat("progress", 0.0)

        self.signal_node.setBool("running", self.running)
        self.last_trigger = trigger
        
    def is_complete(self):
        cur_time = self.imu_node.getFloat("timestamp")
        if cur_time > self.start_time + self.dur_sec:
            comms.events.log("chirp", "complete %.2f rad/sec" % self.freq_end)
            return True
        else:
            return False
    
    def close(self):
        self.active = False
        return True

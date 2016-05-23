from props import root, getNode

import comms.events
from task import Task

class Chirp(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.flight_node = getNode("/controls/flight", True)
        self.imu_node = getNode("/sensors/imu", True)
        self.act_node = getNode("/actuators/actuator", True)
        
        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")
        self.start_time = 0.0
        self.k = 0.0
        # rad/sec is hz*2*pi
        if config_node.hasChild("freq_start_rad_sec"):
            self.freq_start = float(config_node.getString("freq_start_rad_sec"))
            if self.freq_start < 0.1:  self.freq_start = 0.1
            if self.freq_start > 100.0: self.freq_start = 100.0
        else:
            self.freq_start = 6.2830
        if config_node.hasChild("freq_end_rad_sec"):
            self.freq_end = float(config_node.getString("freq_end_rad_sec"))
            if self.freq_end < 0.1:  self.freq_end = 0.1
            if self.freq_end > 100.0: self.freq_end = 100.0
        else:
            self.freq_end = 62.83
        if config_node.hasChild("duration_sec"):
            self.dur_sec = float(config_node.getString("duration_sec"))
            if self.dur_sec < 1:  self.dur_sec = 1
            if self.dur_sec > 100: self.dur_sec = 100
        else:
            self.dur_sec = 20            
        if config_node.hasChild("amplitude_deg"):
            self.ampl_deg = float(config_node.getString("amplitude_deg"))
            if self.ampl_deg < 0.01:  self.ampl_deg = 0.01
            if self.ampl_deg > 60: self.ampl_deg = 60
        else:
            self.ampl_deg = 1            

    def activate(self):
        self.active = True
        comms.events.log("chirp", "start %.2f rad/sec" % self.freq_start)
        self.start_time = self.imu_node.getFloat("timestamp")
        self.k = self.freq_end - self.freq_start / (2 * self.dur_sec)
    
    def update(self):
        if not self.active:
            return False

        cur_time = self.imu_node.getFloat("timestamp")
        t = cur_time - self.start_time
        chirp = self.ampl_deg * math.sin(self.freq_start*t + self.k*t*t)
        self.act_node.setFloat("chirp_deg", chirp)
        
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

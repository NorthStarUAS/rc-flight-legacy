from props import root, getNode

import comms.events
from task import Task

class Preflight(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.task_node = getNode("/task", True)
        self.fcs_node = getNode("/config/autopilot", True)
        self.ap_node = getNode("/autopilot/settings", True)
        self.imu_node = getNode("/sensors/imu", True)
        self.saved_fcs_mode = ""
        self.timer = 0.0
        self.last_imu_timestamp = 0.0
        self.timeout_sec = 60.0
        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")
        self.timeout_sec = config_node.getFloat("timeout_sec")

    def activate(self):
        # fixme, not if airborne!
        self.active = True
        self.saved_fcs_mode = self.fcs_node.getString("mode")
        if not self.task_node.getBool("is_airborne"):
            # set fcs mode to roll+pitch (vanity mode)
            self.fcs_node.setString("mode", "roll+pitch")
            self.ap_node.setFloat( "target_roll_deg", 0.0 )
            self.ap_node.setFloat( "target_pitch_deg", 0.0 )
            # reset timer
            self.timer = 0.0
        else:
            # we are airborne, don't change modes and configure timer
            # to be already expired
            self.timer = self.timeout_sec + 1.0
        self.last_imu_timestamp = self.imu_node.getFloat("timestamp")
        comms.events.log("mission", "preflight")

    def update(self):
        if not self.active:
            return False
        # print "preflight & updating"
        self.imu_timestamp = self.imu_node.getFloat("timestamp")
        dt = self.imu_timestamp - self.last_imu_timestamp
        self.last_imu_timestamp = self.imu_timestamp
        self.timer += dt

    def is_complete(self):
        # print "timer=%.1f timeout=%.1f" % (self.timer, self.timeout_sec)
        # complete when timer expires or we sense we are airborne
        # (sanity check!)
        done = False
        if self.timer >= self.timeout_sec or \
           self.task_node.getBool("is_airborne"):
	    done = True
        return done
    
    def close(self):
        # restore the previous state
        self.fcs_node.setString("mode", self.saved_fcs_mode)
        self.active = False
        return True

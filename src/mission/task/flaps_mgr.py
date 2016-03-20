from props import root, getNode

import comms.events
from task import Task

class FlapsMgr(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.flight_node = getNode("/controls/flight", True)
        self.imu_node = getNode("/sensors/imu", True)
        
        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")
        self.last_time = 0.0
        if config_node.hasChild("speed_secs"):
            self.speed_secs = float(config_node.getString("speed_secs"))
        else:
            self.speed_secs = 5

    def activate(self):
        self.active = True
        comms.events.log("flap_mgr", "active")
        self.last_time = self.imu_node.getFloat("timestamp")
    
    def update(self):
        if not self.active:
            return False

        cur_time = self.imu_node.getFloat("timestamp")
        dt = cur_time - self.last_time
        self.last_time = cur_time
        target_value = self.flight_node.getFloat("flaps_setpoint")
        current_value = self.flight_node.getFloat("flaps")
        df = target_value - current_value
        max = dt / self.speed_secs
        if df > max: df = max
        if df < -max: df = -max
        current_value += df
        self.flight_node.setFloat("flaps", current_value)

    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True

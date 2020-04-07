import math

from props import getNode

import comms.events
from mission.task.task import Task
import mission.task.state

class Circle(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.pos_node = getNode("/position", True)
        self.orient_node = getNode("/orientation", True)
        self.circle_active_node = getNode("/task/circle/active", True)
        self.circle_standby_node = getNode("/task/circle/standby", True)
        self.ap_node = getNode("/autopilot", True)
        self.nav_node = getNode("/navigation", True)
        self.targets_node = getNode("/autopilot/targets", True)

        self.direction = "left"
        self.radius_m = 100.0
        
        self.name = config_node.getString("name")
        if config_node.hasChild("direction"):
            # only override the default if a value is given
            self.direction = config_node.getString("direction")
        if config_node.hasChild("radius_m"):
            # only override the default if a value is given
            self.radius_m = config_node.getFloat("radius_m")

    def update_parameters(self):
        # copy from standby values
        if self.circle_standby_node.hasChild("longitude_deg"):
            lon_deg = self.circle_standby_node.getFloat("longitude_deg")
            if abs(lon_deg) > 0.01:
                self.circle_active_node.setFloat("longitude_deg", lon_deg)
        if self.circle_standby_node.hasChild("latitude_deg"):
            lat_deg = self.circle_standby_node.getFloat("latitude_deg")
            if abs(lat_deg) > 0.01:
                self.circle_active_node.setFloat("latitude_deg", lat_deg)
        if self.circle_standby_node.hasChild("radius_m"):
            radius_m = self.circle_standby_node.getFloat("radius_m")
            if abs(radius_m) > 25:
                self.circle_active_node.setFloat("radius_m", radius_m)
        if self.circle_standby_node.hasChild("direction"):
            dir = self.circle_standby_node.getString("direction")
            if len(dir):
                self.circle_active_node.setString("direction", dir)
            
        
    def activate(self):
        self.active = True

        # save existing state
        mission.task.state.save(modes=True, circle=True, targets=False)
        
        self.update_parameters()
        
        # set modes
        self.ap_node.setString("mode", "basic+tecs")
        self.nav_node.setString("mode", "circle")
        comms.events.log("mission", "circle")
    
    def update(self, dt):
        pass
        
    def is_complete(self):
        return False
    
    def close(self):
        # restore the previous state
        mission.task.state.restore()

        self.active = False
        return True

import math

from PropertyTree import PropertyNode

import comms.events
from mission.task.task import Task
from mission.task import fcsmode
import mission.task.state

class Circle(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.pos_node = PropertyNode("/position")
        self.orient_node = PropertyNode("/orientation")
        self.circle_active_node = PropertyNode("/task/circle/active")
        self.circle_standby_node = PropertyNode("/task/circle/standby")
        self.nav_node = PropertyNode("/navigation")
        self.targets_node = PropertyNode("/autopilot/targets")

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
            lon_deg = self.circle_standby_node.getDouble("longitude_deg")
            if abs(lon_deg) > 0.01:
                self.circle_active_node.setDouble("longitude_deg", lon_deg)
        if self.circle_standby_node.hasChild("latitude_deg"):
            lat_deg = self.circle_standby_node.getDouble("latitude_deg")
            if abs(lat_deg) > 0.01:
                self.circle_active_node.setDouble("latitude_deg", lat_deg)
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

        # save current state
        mission.task.state.save(modes=True, circle=True, targets=False)
        
        self.update_parameters()
        
        # set modes
        fcsmode.set("basic+tecs")
        self.nav_node.setString("mode", "circle")
        comms.events.log("mission", "circle")
    
    def update(self, dt):
        pass
        
    def is_complete(self):
        return False
    
    def close(self):
        # restore previous state
        mission.task.state.restore()

        self.active = False
        return True

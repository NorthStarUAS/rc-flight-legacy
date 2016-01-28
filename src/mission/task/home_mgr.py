from props import root, getNode

import comms.events
from task import Task

class HomeMgr(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.pos_node = getNode("/position", True)
        self.home_node = getNode("/task/home", True)
        self.home_node.setBool("valid", False)
        self.startup_node = getNode("/task/startup", True)
        self.gps_node = getNode("/sensors/gps", True)
        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")

    def activate(self):
        self.active = True
    
    def update(self):
        if not self.active:
            return False
        if not self.home_node.getBool("valid"):
            if self.gps_node.getFloat("gps_age") < 1.0 and \
               self.gps_node.getBool("settle"):
                # Save current position as startup position
                self.startup_node.setFloat("longitude_deg", self.gps_node.getFloat("longitude_deg"))
                self.startup_node.setFloat("latitude_deg", self.gps_node.getFloat("latitude_deg"))
                self.startup_node.setFloat("altitude_m", self.gps_node.getFloat("altitude_m"))
                self.startup_node.setBool("valid", True)

                # Set initial "home" position.
                self.home_node.setFloat("longitude_deg", self.gps_node.getFloat("longitude_deg"))
                self.home_node.setFloat("latitude_deg", self.gps_node.getFloat("latitude_deg"))
                self.home_node.setFloat("altitude_m", self.gps_node.getFloat("altitude_m"))
                self.home_node.setFloat("azimuth_deg", 0.0)
                self.home_node.setBool("valid", True)
        else:
            # FIXME: we could compute distance and bearing to home right here
            pass
        return True
    
    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True

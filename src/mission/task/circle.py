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
        self.circle_node = getNode("/task/circle/active", True)
        self.ap_node = getNode("/autopilot", True)
        self.nav_node = getNode("/navigation", True)
        self.targets_node = getNode("/autopilot/targets", True)

        self.coord_path = ""
        self.direction = "left"
        self.radius_m = 100.0
        self.target_agl_ft = 0.0
        self.target_speed_kt = 0.0
        
        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")
        self.coord_path = config_node.getString("coord_path")
        if config_node.hasChild("direction"):
            # only override the default if a value is given
            self.direction = config_node.getString("direction")
        if config_node.hasChild("radius_m"):
            # only override the default if a value is given
            self.radius_m = config_node.getFloat("radius_m")
        self.target_agl_ft = config_node.getFloat("altitude_agl_ft")
        self.target_speed_kt = config_node.getFloat("speed_kt")

        # this needs to be done at the end if a coord_path is provided
        if self.coord_path != "":
            self.coord_node = getNode(self.coord_path, True)
        else:
            self.coord_node = None

    def activate(self):
        self.active = True

        # save existing state
        have_targets = False
        if self.target_agl_ft > 0 or self.target_speed_kt > 0:
            have_targets = True
        mission.task.state.save(modes=True, circle=True, targets=have_targets)
        
        # override target agl if requested by task
        if self.target_agl_ft > 0.0:
            # self.saved_agl_ft = self.targets_node.getFloat("altitude_agl_ft")
            self.targets_node.setFloat("altitude_agl_ft", self.target_agl_ft)

        # override target speed if requested by task
        if self.target_speed_kt > 0.0:
            # self.saved_speed_kt = self.targets_node.getFloat("airspeed_kt")
            self.targets_node.setFloat("airspeed_kt", self.target_speed_kt)

        # set modes
        self.ap_node.setString("mode", "basic+tecs")
        self.nav_node.setString("mode", "circle")
        comms.events.log("mission", "circle")
    
    def update(self, dt):
        if not self.active:
            return False
        
        # update circle center if task specifies a source/coord path
        if self.coord_node and self.coord_node.hasChild("longitude_deg"):
            self.circle_node.setFloat("longitude_deg",
                                    self.coord_node.getFloat("longitude_deg"))
        if self.coord_node and self.coord_node.hasChild("latitude_deg"):
            self.circle_node.setFloat("latitude_deg",
                                    self.coord_node.getFloat("latitude_deg"))
        
    def is_complete(self):
        return False
    
    def close(self):
        # restore the previous state
        mission.task.state.restore()

        self.active = False
        return True

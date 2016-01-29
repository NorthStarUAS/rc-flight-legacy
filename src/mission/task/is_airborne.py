from props import root, getNode

import comms.events
from task import Task

class IsAirborne(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.pos_node = getNode("/position", True)
        self.vel_node = getNode("/velocity", True)
        self.task_node = getNode("/task", True)
        self.is_airborne = False
        self.off_alt_agl_ft = 0.0
        self.off_airspeed_kt = 0.0
        self.on_alt_agl_ft = 0.0
        self.on_airspeed_kt = 0.0
        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")
        self.off_alt_agl_ft = config_node.getFloat("off_alt_agl_ft")
        self.off_airspeed_kt = config_node.getFloat("off_airspeed_kt")
        self.on_alt_agl_ft = config_node.getFloat("on_alt_agl_ft")
        self.on_airspeed_kt = config_node.getFloat("on_airspeed_kt")

    def activate(self):
        self.active = True
        self.task_node.setBool("is_airborne", False)
        comms.events.log("mission", "on ground")
    
    def update(self):
        if not self.active:
            return False

        if not self.is_airborne:
	    # all conditions must be over the threshold in order to
	    # become airborne
	    cond = True
            if self.off_alt_agl_ft > 0.001 and \
               self.pos_node.getFloat("altitude_agl_ft") < self.off_alt_agl_ft:
	        cond = False
	    if self.off_airspeed_kt > 0.001 and \
               self.vel_node.getFloat("airspeed_kt") < self.off_airspeed_kt:
	        cond = False
	    if cond:
	        self.is_airborne = True
	        self.task_node.setBool("is_airborne", True)
                comms.events.log("mission", "airborne")
        else:
            # if all conditions under their threshold, we are on the ground
            cond = True
            if self.on_alt_agl_ft > 0.001 and \
               self.pos_node.getFloat("altitude_agl_ft") > self.on_alt_agl_ft:
                cond = False
            if self.on_airspeed_kt > 0.001 and \
               self.vel_node.getFloat("airspeed_kt") > self.on_airspeed_kt:
                cond = False
            if cond:
                self.is_airborne = False
                self.task_node.setBool("is_airborne", False)
                comms.events.log("mission", "on ground")

    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True

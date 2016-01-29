from props import root, getNode

import comms.events
from task import Task

class ThrottleSafety(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.act_node = getNode("/actuators", True)
        self.gps_node = getNode("/sensors/gps", True)
        self.task_node = getNode("/task", True)
        
        # initial defaults are locked down (this is kind of a big deal!)
        self.master_safety = True
        self.safety_on_ground = True
        self.act_node.setBool("throttle_safety", True)
        
        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")
        self.safety_on_ground = config_node.getBool("safety_on_ground")

    def activate(self):
        self.active = True
        comms.events.log("safety", "throttle_safety: " + str(self.master_safety))
    
    def update(self):
        if not self.active:
            return False

        if self.master_safety:
            # safety is on, check if we should remove it (so throttle can run.)
            if not self.gps_node.getBool("settle"):
                # do not enable autopilot throttle control if gps
                # hasn't reported a fix and is settled.
                pass
            elif self.safety_on_ground and \
                 not self.task_node.getBool("is_airborne"):
                # safety_on_ground==true means never run throttle
                # unless we are airborne.
                pass
            else:
                self.master_safety = False
                self.act_node.setBool("throttle_safety", self.master_safety)
                comms.events.log("safety", "throttle enabled (safety turned off!)")
        else:
            # safety is off, check if we should turn it on
            if self.safety_on_ground and \
               not self.task_node.getBool("is_airborne"):
                self.master_safety = True
                self.act_node.setBool("throttle_safety", self.master_safety)
                comms.events.log("safety", "Throttle disabled (safety turned on!)")

    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True

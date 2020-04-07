from props import getNode

import comms.events
from mission.task.task import Task

class ThrottleSafety(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.act_node = getNode("/actuators", True)
        self.gps_node = getNode("/sensors/gps", True)
        self.task_node = getNode("/task", True)
        
        # initial defaults are locked down (this is kind of a big deal!)
        self.master_safety = True
        self.safety_mode = 'on_ground'
        self.airborne_latch = False
        self.act_node.setBool("throttle_safety", True)
        
        self.name = config_node.getString("name")
        if config_node.hasChild("safety_mode"):
            self.safety_mode = config_node.getString('safety_mode')

    def activate(self):
        self.active = True
        comms.events.log("safety", "throttle_safety: " + str(self.safety_mode) + " " + str(self.master_safety))
    
    def update(self, dt):
        if not self.active:
            return False

        is_airborne = self.task_node.getBool("is_airborne")
        if not self.airborne_latch and is_airborne:
            self.airborne_latch = True
            
        if self.master_safety:
            # safety is on, check if we should remove it (so throttle can run.)
            if not self.gps_node.getBool("settle"):
                # do not enable autopilot throttle control if gps
                # hasn't reported a fix and is settled.
                pass
            elif self.safety_mode == 'on_ground' and not is_airborne:
                # safety 'on_ground' means never run throttle unless we
                # are airborne.
                pass
            elif self.safety_mode == 'on_touchdown' and not is_airborne and self.airborne_latch:
                # safety 'on_touchdown' means safety starts on, but is
                # switched off after touchdown (assuming some flying happens)
                pass
            else:
                self.master_safety = False
                self.act_node.setBool("throttle_safety", self.master_safety)
                comms.events.log("safety", "throttle enabled (safety turned off!)")
        else:
            # safety is off, check if we should turn it on
            make_safe = False
            if self.safety_mode == 'on_ground' and not is_airborne:
                make_safe = True
            elif self.safety_mode == 'on_touchdown' and self.airborne_latch and not is_airborne:
                make_safe = True
            if make_safe:
                self.master_safety = True
                self.act_node.setBool("throttle_safety", self.master_safety)
                comms.events.log("safety", "Throttle disabled (safety turned on!)")

    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True

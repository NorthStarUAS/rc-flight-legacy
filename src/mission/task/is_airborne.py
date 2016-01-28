from task import Task

class IsAirborne(Task):
    def __init__(self, task_node):
        Task.__init__(self)
        self.is_airborne = False
        self.off_alt_agl_ft = 0.0
        self.off_airspeed_kt = 0.0
        self.on_alt_agl_ft = 0.0
        self.on_airspeed_kt = 0.0
        if task_node.hasChild("name"):
            self.name = task_node.name
        if task_node.hasChild("nickname"):
            self.nickname = task_node.nickname
        if task_node.hasChild("off_alt_agl_ft"):
            self.off_alt_agl_ft = task_node.off_alt_agl_ft
        if task_node.hasChild("off_airspeed_kt"):
            self.off_airspeed_kt = task_node.off_airspeed_kt
        if task_node.hasChild("on_alt_agl_ft"):
            self.on_alt_agl_ft = task_node.on_alt_agl_ft
        if task_node.hasChild("on_airspeed_kt"):
            self.on_airspeed_kt = task_node.on_airspeed_kt
        print "hello from IsAirborne"

    def activate(self):
        self.active = True
        event_log("mission", "On Ground")
    
    def update(self):
        if not self.active:
            return False
        
        if not self.is_airborne:
	    # all conditions must be over the threshold in order to
	    # become airborne
	    cond = True
            if self.off_alt_agl_ft > 0.001 and \
               root.position.altitude_agl_ft < self.off_alt_agl_ft:
	        cond = False
	    if self.off_airspeed_kt > 0.001 and \
               root.velocity.airspeed_kt < self.off_airspeed_kt:
	        cond = False
	    if cond:
	        self.is_airborne = True
	        root.task.is_airborne = True
            event_log("mission", "Airborne")
        else:
            # if all conditions under their threshold, we are on the ground
            cond = True
            if self.on_alt_agl_ft > 0.001 and \
               root.position.altitude_agl_ft > self.on_alt_agl_ft:
                cond = False
            if self.on_airspeed_kt > 0.001 and \
               root.position.airspeed_kt > self.on_airspeed_kt:
                cond = False
            if cond:
                self.is_airborne = False
                root.task.is_airborne = False
                event_log("mission", "On Ground")

    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True

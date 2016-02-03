from props import root, getNode

import comms.events
from task import Task

class Route(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.home_node = getNode("/task/home", True)
        self.route_node = getNode("/task/route", True)
        self.fcs_node = getNode("/autopilot", True)
        self.ap_node = getNode("/autopilot/settings", True)

        self.alt_agl_ft = 0.0
        self.speed_kt = 30.0

        self.last_lon = 0.0
        self.last_lat = 0.0
        self.last_az = 0.0
        
        self.saved_fcs_mode = ""
        self.saved_agl_ft = 0.0
        self.saved_speed_kt = 0.0

        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")
        self.coord_path = config_node.getString("coord_path")
        self.alt_agl_ft = config_node.getFloat("altitude_agl_ft")
        self.speed_kt = config_node.getFloat("speed_kt")

    def activate(self):
        self.active = True

        # save existing state
        self.saved_fcs_mode = self.fcs_node.getString("mode")
        self.saved_agl_ft = self.ap_node.getFloat("target_agl_ft")
        self.saved_speed_kt = self.ap_node.getFloat("target_speed_kt")

        # set fcs mode to basic+alt+speed
        self.fcs_node.setString("mode", "basic+alt+speed")

        if self.alt_agl_ft > 0.1:
            self.ap_node.setFloat("target_agl_ft", self.alt_agl_ft)

        self.route_node.setString("follow_mode", "leader");
        self.route_node.setString("start_mode", "first_wpt");
        self.route_node.setString("completion_mode", "loop");
        
        comms.events.log("mission", "route")
    
    def update(self):
        if not self.active:
            return False
        # route_mgr update (code to fly the actual route) is written
        # in C++ and located in src/control/route_mgr.cxx The
        # route_mgr->update() routine is called from
        # src/control/control.cxx:update() whenever a route following
        # task is active.
        
    def is_complete(self):
        return False
    
    def close(self):
        # restore the previous state
        self.fcs_node.setString("mode", self.saved_fcs_mode)
        self.ap_node.setFloat("target_agl_ft", self.saved_agl_ft)
        self.ap_node.setFloat("target_speed_kt", self.saved_speed_kt)

        self.active = False
        return True

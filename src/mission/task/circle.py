from props import root, getNode

import comms.events
from task import Task

class Circle(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.pos_node = getNode("/position", True)
        self.orient_node = getNode("/orientation", True)
        self.task_node = getNode("/task/circle", True)
        self.fcs_node = getNode("/config/fcs", True)
        self.ap_node = getNode("/autopilot/settings", True)

        self.coord_path = ""
        self.direction = "left"
        self.radius_m = 100.0
        self.target_agl_ft = 0.0
        self.target_speed_kt = 0.0
        
        self.saved_fcs_mode = ""
        self.saved_lon_deg = 0.0
        self.saved_lat_deg = 0.0
        self.saved_direction = ""
        self.saved_radius_m = 0.0
        self.saved_agl_ft = 0.0
        self.saved_speed_kt = 0.0

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
        self.saved_fcs_mode = self.fcs_node.getString("mode")
        self.saved_lon_deg = self.task_node.getFloat("longitude_deg")
        self.saved_lat_deg = self.task_node.getFloat("latitude_deg")

        self.saved_direction = self.task_node.getString("direction")
        self.task_node.setString("direction", self.direction)

        self.saved_radius_m = self.task_node.getFloat("radius_m")
        self.task_node.setFloat("radius_m", self.radius_m)

        # override target agl if requested by task
        if self.target_agl_ft > 0.0:
            self.saved_agl_ft = self.ap_node.getFloat("target_agl_ft")
            self.ap_node.setFloat("target_agl_ft", self.target_agl_ft)

        # override target speed if requested by task
        if self.target_speed_kt > 0.0:
            self.saved_speed_kt = self.ap_node.getFloat("target_speed_kt")
            self.ap_node.setFloat("target_speed_kt", self.target_speed_kt)

        # set fcs mode to basic+alt+speed
        self.fcs_node.setString("mode", "basic+alt+speed")
        comms.events.log("mission", "circle")
    
    def update(self):
        if not self.active:
            return False
        # update circle center if task specifies a source/coord path
        if self.coord_node and self.coord_node.hasChild("longitude_deg"):
            self.task_node.setFloat("longitude_deg",
                                    self.coord_node.getFloat("longitude_deg"))
        if self.coord_node and self.coord_node.hasChild("latitude_deg"):
            self.circle_node.setFloat("latitude_deg",
                                      self.coord_node.getFloat("latitude_deg"))
        # circle_mgr update (code to fly the actual circle) is C++
        # code and located in src/control/ The circle_mgr->update()
        # routine is called from src/control/control.cxx:update()
        # whenever a circle hold task is active.
        
    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True

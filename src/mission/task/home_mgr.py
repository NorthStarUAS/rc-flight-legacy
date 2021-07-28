# home_mgr.py: set the home location to the current location as soon
# as there is valid information to do this.  Compute distance,
# heading, and x, y components to home.

import math

from PropertyTree import PropertyNode
from rcUAS import wgs84

import comms.events
from mission.task.task import Task

d2r = math.pi / 180.0

class HomeMgr(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.pos_node = PropertyNode("/position")
        self.home_node = PropertyNode("/task/home")
        self.home_node.setBool("valid", False)
        self.startup_node = PropertyNode("/task/startup")
        self.gps_node = PropertyNode("/sensors/gps/0")
        self.name = config_node.getString("name")

    def activate(self):
        self.active = True
    
    def update(self, dt):
        if not self.active:
            return False
        if not self.home_node.getBool("valid"):
            if self.gps_node.getDouble("gps_age") < 1.0 and \
               self.gps_node.getBool("settle"):
                # Save current position as startup position
                self.startup_node.setDouble("longitude_deg", self.gps_node.getDouble("longitude_deg"))
                self.startup_node.setDouble("latitude_deg", self.gps_node.getDouble("latitude_deg"))
                self.startup_node.setDouble("altitude_m", self.gps_node.getDouble("altitude_m"))
                self.startup_node.setBool("valid", True)

                # Set initial "home" position.
                self.home_node.setDouble("longitude_deg", self.gps_node.getDouble("longitude_deg"))
                self.home_node.setDouble("latitude_deg", self.gps_node.getDouble("latitude_deg"))
                self.home_node.setDouble("altitude_m", self.gps_node.getDouble("altitude_m"))
                self.home_node.setDouble("azimuth_deg", 0.0)
                self.home_node.setBool("valid", True)
        else:
            current = (self.pos_node.getDouble("latitude_deg"),
                       self.pos_node.getDouble("longitude_deg"))
            home = (self.home_node.getDouble("latitude_deg"),
                    self.home_node.getDouble("longitude_deg"))
            (course_deg, rev_deg, dist_m) = \
                wgs84.geo_inverse( self.pos_node.getDouble("latitude_deg"),
                                   self.pos_node.getDouble("longitude_deg"),
                                   self.home_node.getDouble("latitude_deg"),
                                   self.home_node.getDouble("longitude_deg") )
            self.home_node.setDouble("course_deg", course_deg)
            self.home_node.setDouble("dist_m", dist_m)
            
            # a mini cartesian (2d) system relative to home in meters
            
            # FIXME: the parametric task is the only thing that uses this
            # so why not put this code over into the parametric.py module?
            
            theta = rev_deg * d2r
            x = math.sin(theta) * dist_m
            y = math.cos(theta) * dist_m
            self.home_node.setDouble("x_m", x)
            self.home_node.setDouble("y_m", y)
        return True
    
    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True

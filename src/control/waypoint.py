from props import getNode
from auracore import wgs84

class Waypoint:
    def __init__(self):
        self.mode = None
        self.lon_deg = 0.0
        self.lat_deg = 0.0
        self.hdg_deg = 0.0
        self.dist_m = 0.0
        self.leg_dist_m = 0.0

    def build(self, config):
        if config.hasChild("lon_deg"):
            self.lon_deg = config.getFloat("lon_deg")
            self.mode = 'absolute'
        if config.hasChild("lat_deg"):
            self.lat_deg = config.getFloat("lat_deg")
            self.mode = 'absolute'
        if config.hasChild("heading_deg"):
            self.hdg_deg = config.getFloat("heading_deg")
            self.mode = 'relative'
        if config.hasChild("dist_m"):
            self.dist_m = config.getFloat("dist_m")
            self.mode = 'relative'
        if self.mode == None:
            print("Error in route waypoint config logic:")
            config.pretty_print("  ")
        elif self.mode == 'absolute':
            print("WPT: %.8f %.8f" % (self.lon_deg, self.lat_deg))
        elif self.mode == 'relative':
            print("WPT: %4.0f deg %.0f m" % (self.hdg_deg, self.dist_m))

    def update_relative_pos(self, lon_deg, lat_deg, ref_heading_deg):
        if self.mode == 'relative':
            course = ref_heading_deg + self.hdg_deg
            if course < 0.0: course += 360.0
            if course > 360.0: course -= 360.0
            (self.lat_deg, self.lon_deg, az2) = \
                wgs84.geo_direct( lat_deg, lon_deg, course, self.dist_m )
        else:
            print("Error: cannot update relative position of absolute waypoint")

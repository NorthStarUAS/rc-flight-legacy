import struct

# Message id constants
simple_test_id = 0
array_test_id = 1
gps_v4_id = 34

# Message: simple_test
# Id: 0
# Packed message size: 2
class simple_test():
    id = 0
    len = 2
    pack_string = "<h"

    def __init__(self, msg=None):
        self.a = 0
        if msg:
            self.unpack(msg)

    def pack(self):
        msg = struct.pack(self.pack_string,
                          self.a)
        return msg

    def unpack(self, msg):
        (self.a,) = struct.unpack(self.pack_string, msg)

# Message: array_test
# Id: 1
# Packed message size: 2
class array_test():
    id = 1
    len = 2
    pack_string = "<h"

    def __init__(self, msg=None):
        self.orientation[9] = 0.0
        if msg:
            self.unpack(msg)

    def pack(self):
        msg = struct.pack(self.pack_string,
                          int(round(self.orientation[9] * 53.3)))
        return msg

    def unpack(self, msg):
        (self.orientation[9],) = struct.unpack(self.pack_string, msg)
        self.orientation[9] /= 53.3

# Message: gps_v4
# Id: 34
# Packed message size: 47
class gps_v4():
    id = 34
    len = 47
    pack_string = "<BfddfhhhdBHHHB"

    def __init__(self, msg=None):
        self.index = 0
        self.time_sec = 0.0
        self.latitude_deg = 0.0
        self.longitude_deg = 0.0
        self.altitude_m = 0.0
        self.vn_ms = 0.0
        self.ve_ms = 0.0
        self.vd_ms = 0.0
        self.unixtime_sec = 0.0
        self.satellites = 0
        self.horiz_accuracy_m = 0.0
        self.vert_accuracy_m = 0.0
        self.pdop = 0.0
        self.fix_type = 0
        if msg:
            self.unpack(msg)

    def pack(self):
        msg = struct.pack(self.pack_string,
                          self.index,
                          self.time_sec,
                          self.latitude_deg,
                          self.longitude_deg,
                          self.altitude_m,
                          int(round(self.vn_ms * 100)),
                          int(round(self.ve_ms * 100)),
                          int(round(self.vd_ms * 100)),
                          self.unixtime_sec,
                          self.satellites,
                          int(round(self.horiz_accuracy_m * 100)),
                          int(round(self.vert_accuracy_m * 100)),
                          int(round(self.pdop * 100)),
                          self.fix_type)
        return msg

    def unpack(self, msg):
        (self.index,
         self.time_sec,
         self.latitude_deg,
         self.longitude_deg,
         self.altitude_m,
         self.vn_ms,
         self.ve_ms,
         self.vd_ms,
         self.unixtime_sec,
         self.satellites,
         self.horiz_accuracy_m,
         self.vert_accuracy_m,
         self.pdop,
         self.fix_type) = struct.unpack(self.pack_string, msg)
        self.vn_ms /= 100
        self.ve_ms /= 100
        self.vd_ms /= 100
        self.horiz_accuracy_m /= 100
        self.vert_accuracy_m /= 100
        self.pdop /= 100


import struct

# Message id constants
simple_test_id = 0
array_test_id = 1
gps_v4_id = 34

# Message: simple_test
# Id: 0
class simple_test():
    id = 0
    _pack_string = "<h"

    def __init__(self, msg=None):
        # public fields
        self.a = 0
        # optional
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.a)
        return msg

    def unpack(self, msg):
        (self.a,) = struct.unpack(self._pack_string, msg)

# Message: array_test
# Id: 1
class array_test():
    id = 1
    _pack_string = "<dbbbbhhhhhhhhhH"

    def __init__(self, msg=None):
        # public fields
        self.time = 0.0
        self.flags = [0] * 4
        self.orientation = [0.0] * 9
        self.something = 0
        # optional
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.time,
                          self.flags[0],
                          self.flags[1],
                          self.flags[2],
                          self.flags[3],
                          int(round(self.orientation[0] * 53.3)),
                          int(round(self.orientation[1] * 53.3)),
                          int(round(self.orientation[2] * 53.3)),
                          int(round(self.orientation[3] * 53.3)),
                          int(round(self.orientation[4] * 53.3)),
                          int(round(self.orientation[5] * 53.3)),
                          int(round(self.orientation[6] * 53.3)),
                          int(round(self.orientation[7] * 53.3)),
                          int(round(self.orientation[8] * 53.3)),
                          self.something)
        return msg

    def unpack(self, msg):
        (self.time,
         self.flags[0],
         self.flags[1],
         self.flags[2],
         self.flags[3],
         self.orientation[0],
         self.orientation[1],
         self.orientation[2],
         self.orientation[3],
         self.orientation[4],
         self.orientation[5],
         self.orientation[6],
         self.orientation[7],
         self.orientation[8],
         self.something) = struct.unpack(self._pack_string, msg)
        self.orientation[0] /= 53.3
        self.orientation[1] /= 53.3
        self.orientation[2] /= 53.3
        self.orientation[3] /= 53.3
        self.orientation[4] /= 53.3
        self.orientation[5] /= 53.3
        self.orientation[6] /= 53.3
        self.orientation[7] /= 53.3
        self.orientation[8] /= 53.3

# Message: gps_v4
# Id: 34
class gps_v4():
    id = 34
    _pack_string = "<BfddfhhhdBHHHB"

    def __init__(self, msg=None):
        # public fields
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
        # optional
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
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
         self.fix_type) = struct.unpack(self._pack_string, msg)
        self.vn_ms /= 100
        self.ve_ms /= 100
        self.vd_ms /= 100
        self.horiz_accuracy_m /= 100
        self.vert_accuracy_m /= 100
        self.pdop /= 100


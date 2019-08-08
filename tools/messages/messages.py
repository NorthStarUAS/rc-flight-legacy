import struct

# Message id constants
simple_test_id = 0
array_test_id = 1
dynamic_string_test_id = 2
enum_test_id = 3
gps_v4_id = 34

# Constants
max_flags = 4  # flags
max_args = 4  # args

# Enums
enum_sequence1_enum1 = 0
enum_sequence1_enum2 = 1  # None
enum_sequence1_enum3 = 2
enum_sequence1_enum4 = 3
enum_sequence1_enum5 = 4
enum_sequence2_enum1a = 0
enum_sequence2_enum2a = 1
enum_sequence2_enum3a = 2
enum_sequence2_enum4a = 3
enum_sequence2_enum5a = 4

# Message: simple_test
# Id: 0
class simple_test():
    id = 0
    _pack_string = "<h"

    def __init__(self, msg=None):
        # public fields
        self.a = 0
        # unpack if requested
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
        self.flags = [0] * max_flags
        self.orientation = [0.0] * 9
        self.something = 0
        # unpack if requested
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

# Message: dynamic_string_test
# Id: 2
class dynamic_string_test():
    id = 2
    _pack_string = "<dBHBBBBB"

    def __init__(self, msg=None):
        # public fields
        self.time = 0.0
        self.event = ""
        self.counter = 0
        self.args = [""] * max_args
        self.status = False
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.time,
                          len(self.event),
                          self.counter,
                          len(self.args[0]),
                          len(self.args[1]),
                          len(self.args[2]),
                          len(self.args[3]),
                          self.status)
        msg += str.encode(self.event)
        msg += str.encode(self.args[0])
        msg += str.encode(self.args[1])
        msg += str.encode(self.args[2])
        msg += str.encode(self.args[3])
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        self.args_len = [0] * max_args
        (self.time,
         self.event_len,
         self.counter,
         self.args_len[0],
         self.args_len[1],
         self.args_len[2],
         self.args_len[3],
         self.status) = struct.unpack(self._pack_string, msg)
        self.event = extra[:self.event_len].decode()
        extra = extra[self.event_len:]
        self.args[0] = extra[:self.args_len[0]].decode()
        extra = extra[self.args_len[0]:]
        self.args[1] = extra[:self.args_len[1]].decode()
        extra = extra[self.args_len[1]:]
        self.args[2] = extra[:self.args_len[2]].decode()
        extra = extra[self.args_len[2]:]
        self.args[3] = extra[:self.args_len[3]].decode()
        extra = extra[self.args_len[3]:]

# Message: enum_test
# Id: 3
class enum_test():
    id = 3
    _pack_string = "<B"

    def __init__(self, msg=None):
        # public fields
        self.time = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.time)
        return msg

    def unpack(self, msg):
        (self.time,) = struct.unpack(self._pack_string, msg)

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
        # unpack if requested
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


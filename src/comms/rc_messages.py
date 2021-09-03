import struct
from PropertyTree import PropertyNode

# Message id constants
gps_v3_id = 26
gps_v4_id = 34
gps_v5_id = 49
gps_raw_v1_id = 48
imu_v4_id = 35
imu_v5_id = 45
imu_v6_id = 50
airdata_v5_id = 18
airdata_v6_id = 40
airdata_v7_id = 43
filter_v3_id = 31
filter_v4_id = 36
filter_v5_id = 47
actuator_v2_id = 21
actuator_v3_id = 37
pilot_v2_id = 20
pilot_v3_id = 38
ap_status_v4_id = 30
ap_status_v5_id = 32
ap_status_v6_id = 33
ap_status_v7_id = 39
system_health_v4_id = 19
system_health_v5_id = 41
system_health_v6_id = 46
payload_v2_id = 23
payload_v3_id = 42
event_v1_id = 27
event_v2_id = 44
command_v1_id = 28

# Constants
max_raw_sats = 12  # maximum array size to store satellite raw data

# Message: gps_v3
# Id: 26
class gps_v3():
    id = 26
    _pack_string = "<BdddfhhhdBHHHB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
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
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  self.latitude_deg,
                  self.longitude_deg,
                  self.altitude_m,
                  int(round(self.vn_ms * 100.0)),
                  int(round(self.ve_ms * 100.0)),
                  int(round(self.vd_ms * 100.0)),
                  self.unixtime_sec,
                  self.satellites,
                  int(round(self.horiz_accuracy_m * 100.0)),
                  int(round(self.vert_accuracy_m * 100.0)),
                  int(round(self.pdop * 100.0)),
                  self.fix_type)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
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
         self.fix_type) = self._struct.unpack(msg)
        self.vn_ms /= 100.0
        self.ve_ms /= 100.0
        self.vd_ms /= 100.0
        self.horiz_accuracy_m /= 100.0
        self.vert_accuracy_m /= 100.0
        self.pdop /= 100.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("latitude_deg", self.latitude_deg)
        node.setDouble("longitude_deg", self.longitude_deg)
        node.setDouble("altitude_m", self.altitude_m)
        node.setDouble("vn_ms", self.vn_ms)
        node.setDouble("ve_ms", self.ve_ms)
        node.setDouble("vd_ms", self.vd_ms)
        node.setDouble("unixtime_sec", self.unixtime_sec)
        node.setUInt("satellites", self.satellites)
        node.setDouble("horiz_accuracy_m", self.horiz_accuracy_m)
        node.setDouble("vert_accuracy_m", self.vert_accuracy_m)
        node.setDouble("pdop", self.pdop)
        node.setUInt("fix_type", self.fix_type)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        latitude_deg = node.getDouble("latitude_deg")
        longitude_deg = node.getDouble("longitude_deg")
        altitude_m = node.getDouble("altitude_m")
        vn_ms = node.getDouble("vn_ms")
        ve_ms = node.getDouble("ve_ms")
        vd_ms = node.getDouble("vd_ms")
        unixtime_sec = node.getDouble("unixtime_sec")
        satellites = node.getUInt("satellites")
        horiz_accuracy_m = node.getDouble("horiz_accuracy_m")
        vert_accuracy_m = node.getDouble("vert_accuracy_m")
        pdop = node.getDouble("pdop")
        fix_type = node.getUInt("fix_type")

# Message: gps_v4
# Id: 34
class gps_v4():
    id = 34
    _pack_string = "<BfddfhhhdBHHHB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
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
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  self.latitude_deg,
                  self.longitude_deg,
                  self.altitude_m,
                  int(round(self.vn_ms * 100.0)),
                  int(round(self.ve_ms * 100.0)),
                  int(round(self.vd_ms * 100.0)),
                  self.unixtime_sec,
                  self.satellites,
                  int(round(self.horiz_accuracy_m * 100.0)),
                  int(round(self.vert_accuracy_m * 100.0)),
                  int(round(self.pdop * 100.0)),
                  self.fix_type)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
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
         self.fix_type) = self._struct.unpack(msg)
        self.vn_ms /= 100.0
        self.ve_ms /= 100.0
        self.vd_ms /= 100.0
        self.horiz_accuracy_m /= 100.0
        self.vert_accuracy_m /= 100.0
        self.pdop /= 100.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("latitude_deg", self.latitude_deg)
        node.setDouble("longitude_deg", self.longitude_deg)
        node.setDouble("altitude_m", self.altitude_m)
        node.setDouble("vn_ms", self.vn_ms)
        node.setDouble("ve_ms", self.ve_ms)
        node.setDouble("vd_ms", self.vd_ms)
        node.setDouble("unixtime_sec", self.unixtime_sec)
        node.setUInt("satellites", self.satellites)
        node.setDouble("horiz_accuracy_m", self.horiz_accuracy_m)
        node.setDouble("vert_accuracy_m", self.vert_accuracy_m)
        node.setDouble("pdop", self.pdop)
        node.setUInt("fix_type", self.fix_type)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        latitude_deg = node.getDouble("latitude_deg")
        longitude_deg = node.getDouble("longitude_deg")
        altitude_m = node.getDouble("altitude_m")
        vn_ms = node.getDouble("vn_ms")
        ve_ms = node.getDouble("ve_ms")
        vd_ms = node.getDouble("vd_ms")
        unixtime_sec = node.getDouble("unixtime_sec")
        satellites = node.getUInt("satellites")
        horiz_accuracy_m = node.getDouble("horiz_accuracy_m")
        vert_accuracy_m = node.getDouble("vert_accuracy_m")
        pdop = node.getDouble("pdop")
        fix_type = node.getUInt("fix_type")

# Message: gps_v5
# Id: 49
class gps_v5():
    id = 49
    _pack_string = "<BLQBBllfhhhhhhh"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.millis = 0
        self.unix_usec = 0
        self.num_sats = 0
        self.status = 0
        self.longitude_raw = 0
        self.latitude_raw = 0
        self.altitude_m = 0.0
        self.vn_mps = 0.0
        self.ve_mps = 0.0
        self.vd_mps = 0.0
        self.hAcc_m = 0.0
        self.vAcc_m = 0.0
        self.hdop = 0.0
        self.vdop = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.millis,
                  self.unix_usec,
                  self.num_sats,
                  self.status,
                  self.longitude_raw,
                  self.latitude_raw,
                  self.altitude_m,
                  int(round(self.vn_mps * 100.0)),
                  int(round(self.ve_mps * 100.0)),
                  int(round(self.vd_mps * 100.0)),
                  int(round(self.hAcc_m * 100.0)),
                  int(round(self.vAcc_m * 100.0)),
                  int(round(self.hdop * 100.0)),
                  int(round(self.vdop * 100.0)))
        return msg

    def unpack(self, msg):
        (self.index,
         self.millis,
         self.unix_usec,
         self.num_sats,
         self.status,
         self.longitude_raw,
         self.latitude_raw,
         self.altitude_m,
         self.vn_mps,
         self.ve_mps,
         self.vd_mps,
         self.hAcc_m,
         self.vAcc_m,
         self.hdop,
         self.vdop) = self._struct.unpack(msg)
        self.vn_mps /= 100.0
        self.ve_mps /= 100.0
        self.vd_mps /= 100.0
        self.hAcc_m /= 100.0
        self.vAcc_m /= 100.0
        self.hdop /= 100.0
        self.vdop /= 100.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setUInt("millis", self.millis)
        node.setUInt64("unix_usec", self.unix_usec)
        node.setUInt("num_sats", self.num_sats)
        node.setUInt("status", self.status)
        node.setInt("longitude_raw", self.longitude_raw)
        node.setInt("latitude_raw", self.latitude_raw)
        node.setDouble("altitude_m", self.altitude_m)
        node.setDouble("vn_mps", self.vn_mps)
        node.setDouble("ve_mps", self.ve_mps)
        node.setDouble("vd_mps", self.vd_mps)
        node.setDouble("hAcc_m", self.hAcc_m)
        node.setDouble("vAcc_m", self.vAcc_m)
        node.setDouble("hdop", self.hdop)
        node.setDouble("vdop", self.vdop)

    def props2msg(self, node):
        index = node.getUInt("index")
        millis = node.getUInt("millis")
        unix_usec = node.getUInt64("unix_usec")
        num_sats = node.getUInt("num_sats")
        status = node.getUInt("status")
        longitude_raw = node.getInt("longitude_raw")
        latitude_raw = node.getInt("latitude_raw")
        altitude_m = node.getDouble("altitude_m")
        vn_mps = node.getDouble("vn_mps")
        ve_mps = node.getDouble("ve_mps")
        vd_mps = node.getDouble("vd_mps")
        hAcc_m = node.getDouble("hAcc_m")
        vAcc_m = node.getDouble("vAcc_m")
        hdop = node.getDouble("hdop")
        vdop = node.getDouble("vdop")

# Message: gps_raw_v1
# Id: 48
class gps_raw_v1():
    id = 48
    _pack_string = "<BfdBBBBBBBBBBBBBdddddddddddddddddddddddd"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.receiver_tow = 0.0
        self.num_sats = 0
        self.svid = [0] * max_raw_sats
        self.pseudorange = [0.0] * max_raw_sats
        self.doppler = [0.0] * max_raw_sats
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  self.receiver_tow,
                  self.num_sats,
                  self.svid[0],
                  self.svid[1],
                  self.svid[2],
                  self.svid[3],
                  self.svid[4],
                  self.svid[5],
                  self.svid[6],
                  self.svid[7],
                  self.svid[8],
                  self.svid[9],
                  self.svid[10],
                  self.svid[11],
                  self.pseudorange[0],
                  self.pseudorange[1],
                  self.pseudorange[2],
                  self.pseudorange[3],
                  self.pseudorange[4],
                  self.pseudorange[5],
                  self.pseudorange[6],
                  self.pseudorange[7],
                  self.pseudorange[8],
                  self.pseudorange[9],
                  self.pseudorange[10],
                  self.pseudorange[11],
                  self.doppler[0],
                  self.doppler[1],
                  self.doppler[2],
                  self.doppler[3],
                  self.doppler[4],
                  self.doppler[5],
                  self.doppler[6],
                  self.doppler[7],
                  self.doppler[8],
                  self.doppler[9],
                  self.doppler[10],
                  self.doppler[11])
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.receiver_tow,
         self.num_sats,
         self.svid[0],
         self.svid[1],
         self.svid[2],
         self.svid[3],
         self.svid[4],
         self.svid[5],
         self.svid[6],
         self.svid[7],
         self.svid[8],
         self.svid[9],
         self.svid[10],
         self.svid[11],
         self.pseudorange[0],
         self.pseudorange[1],
         self.pseudorange[2],
         self.pseudorange[3],
         self.pseudorange[4],
         self.pseudorange[5],
         self.pseudorange[6],
         self.pseudorange[7],
         self.pseudorange[8],
         self.pseudorange[9],
         self.pseudorange[10],
         self.pseudorange[11],
         self.doppler[0],
         self.doppler[1],
         self.doppler[2],
         self.doppler[3],
         self.doppler[4],
         self.doppler[5],
         self.doppler[6],
         self.doppler[7],
         self.doppler[8],
         self.doppler[9],
         self.doppler[10],
         self.doppler[11]) = self._struct.unpack(msg)

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("receiver_tow", self.receiver_tow)
        node.setUInt("num_sats", self.num_sats)
        for _i in range(max_raw_sats): node.setUInt("svid", self.svid[_i], _i)
        for _i in range(max_raw_sats): node.setDouble("pseudorange", self.pseudorange[_i], _i)
        for _i in range(max_raw_sats): node.setDouble("doppler", self.doppler[_i], _i)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        receiver_tow = node.getDouble("receiver_tow")
        num_sats = node.getUInt("num_sats")
        for _i in range(max_raw_sats): svid[_i] = node.getUInt("svid", _i)
        for _i in range(max_raw_sats): pseudorange[_i] = node.getDouble("pseudorange", _i)
        for _i in range(max_raw_sats): doppler[_i] = node.getDouble("doppler", _i)

# Message: imu_v4
# Id: 35
class imu_v4():
    id = 35
    _pack_string = "<BffffffffffhB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.p_rad_sec = 0.0
        self.q_rad_sec = 0.0
        self.r_rad_sec = 0.0
        self.ax_mps_sec = 0.0
        self.ay_mps_sec = 0.0
        self.az_mps_sec = 0.0
        self.hx = 0.0
        self.hy = 0.0
        self.hz = 0.0
        self.temp_C = 0.0
        self.status = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  self.p_rad_sec,
                  self.q_rad_sec,
                  self.r_rad_sec,
                  self.ax_mps_sec,
                  self.ay_mps_sec,
                  self.az_mps_sec,
                  self.hx,
                  self.hy,
                  self.hz,
                  int(round(self.temp_C * 10.0)),
                  self.status)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.p_rad_sec,
         self.q_rad_sec,
         self.r_rad_sec,
         self.ax_mps_sec,
         self.ay_mps_sec,
         self.az_mps_sec,
         self.hx,
         self.hy,
         self.hz,
         self.temp_C,
         self.status) = self._struct.unpack(msg)
        self.temp_C /= 10.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("p_rad_sec", self.p_rad_sec)
        node.setDouble("q_rad_sec", self.q_rad_sec)
        node.setDouble("r_rad_sec", self.r_rad_sec)
        node.setDouble("ax_mps_sec", self.ax_mps_sec)
        node.setDouble("ay_mps_sec", self.ay_mps_sec)
        node.setDouble("az_mps_sec", self.az_mps_sec)
        node.setDouble("hx", self.hx)
        node.setDouble("hy", self.hy)
        node.setDouble("hz", self.hz)
        node.setDouble("temp_C", self.temp_C)
        node.setUInt("status", self.status)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        p_rad_sec = node.getDouble("p_rad_sec")
        q_rad_sec = node.getDouble("q_rad_sec")
        r_rad_sec = node.getDouble("r_rad_sec")
        ax_mps_sec = node.getDouble("ax_mps_sec")
        ay_mps_sec = node.getDouble("ay_mps_sec")
        az_mps_sec = node.getDouble("az_mps_sec")
        hx = node.getDouble("hx")
        hy = node.getDouble("hy")
        hz = node.getDouble("hz")
        temp_C = node.getDouble("temp_C")
        status = node.getUInt("status")

# Message: imu_v5
# Id: 45
class imu_v5():
    id = 45
    _pack_string = "<BffffffffffffffffhB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.p_rad_sec = 0.0
        self.q_rad_sec = 0.0
        self.r_rad_sec = 0.0
        self.ax_mps_sec = 0.0
        self.ay_mps_sec = 0.0
        self.az_mps_sec = 0.0
        self.hx = 0.0
        self.hy = 0.0
        self.hz = 0.0
        self.ax_raw = 0.0
        self.ay_raw = 0.0
        self.az_raw = 0.0
        self.hx_raw = 0.0
        self.hy_raw = 0.0
        self.hz_raw = 0.0
        self.temp_C = 0.0
        self.status = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  self.p_rad_sec,
                  self.q_rad_sec,
                  self.r_rad_sec,
                  self.ax_mps_sec,
                  self.ay_mps_sec,
                  self.az_mps_sec,
                  self.hx,
                  self.hy,
                  self.hz,
                  self.ax_raw,
                  self.ay_raw,
                  self.az_raw,
                  self.hx_raw,
                  self.hy_raw,
                  self.hz_raw,
                  int(round(self.temp_C * 10.0)),
                  self.status)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.p_rad_sec,
         self.q_rad_sec,
         self.r_rad_sec,
         self.ax_mps_sec,
         self.ay_mps_sec,
         self.az_mps_sec,
         self.hx,
         self.hy,
         self.hz,
         self.ax_raw,
         self.ay_raw,
         self.az_raw,
         self.hx_raw,
         self.hy_raw,
         self.hz_raw,
         self.temp_C,
         self.status) = self._struct.unpack(msg)
        self.temp_C /= 10.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("p_rad_sec", self.p_rad_sec)
        node.setDouble("q_rad_sec", self.q_rad_sec)
        node.setDouble("r_rad_sec", self.r_rad_sec)
        node.setDouble("ax_mps_sec", self.ax_mps_sec)
        node.setDouble("ay_mps_sec", self.ay_mps_sec)
        node.setDouble("az_mps_sec", self.az_mps_sec)
        node.setDouble("hx", self.hx)
        node.setDouble("hy", self.hy)
        node.setDouble("hz", self.hz)
        node.setDouble("ax_raw", self.ax_raw)
        node.setDouble("ay_raw", self.ay_raw)
        node.setDouble("az_raw", self.az_raw)
        node.setDouble("hx_raw", self.hx_raw)
        node.setDouble("hy_raw", self.hy_raw)
        node.setDouble("hz_raw", self.hz_raw)
        node.setDouble("temp_C", self.temp_C)
        node.setUInt("status", self.status)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        p_rad_sec = node.getDouble("p_rad_sec")
        q_rad_sec = node.getDouble("q_rad_sec")
        r_rad_sec = node.getDouble("r_rad_sec")
        ax_mps_sec = node.getDouble("ax_mps_sec")
        ay_mps_sec = node.getDouble("ay_mps_sec")
        az_mps_sec = node.getDouble("az_mps_sec")
        hx = node.getDouble("hx")
        hy = node.getDouble("hy")
        hz = node.getDouble("hz")
        ax_raw = node.getDouble("ax_raw")
        ay_raw = node.getDouble("ay_raw")
        az_raw = node.getDouble("az_raw")
        hx_raw = node.getDouble("hx_raw")
        hy_raw = node.getDouble("hy_raw")
        hz_raw = node.getDouble("hz_raw")
        temp_C = node.getDouble("temp_C")
        status = node.getUInt("status")

# Message: imu_v6
# Id: 50
class imu_v6():
    id = 50
    _pack_string = "<BLhhhhhhhhhhhhhhhh"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.millis = 0
        self.ax_raw = 0.0
        self.ay_raw = 0.0
        self.az_raw = 0.0
        self.hx_raw = 0.0
        self.hy_raw = 0.0
        self.hz_raw = 0.0
        self.ax_mps2 = 0.0
        self.ay_mps2 = 0.0
        self.az_mps2 = 0.0
        self.p_rps = 0.0
        self.q_rps = 0.0
        self.r_rps = 0.0
        self.hx = 0.0
        self.hy = 0.0
        self.hz = 0.0
        self.temp_C = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.millis,
                  int(round(self.ax_raw * 835.296217)),
                  int(round(self.ay_raw * 835.296217)),
                  int(round(self.az_raw * 835.296217)),
                  int(round(self.hx_raw * 30000.0)),
                  int(round(self.hy_raw * 30000.0)),
                  int(round(self.hz_raw * 30000.0)),
                  int(round(self.ax_mps2 * 835.296217)),
                  int(round(self.ay_mps2 * 835.296217)),
                  int(round(self.az_mps2 * 835.296217)),
                  int(round(self.p_rps * 3754.82165)),
                  int(round(self.q_rps * 3754.82165)),
                  int(round(self.r_rps * 3754.82165)),
                  int(round(self.hx * 30000.0)),
                  int(round(self.hy * 30000.0)),
                  int(round(self.hz * 30000.0)),
                  int(round(self.temp_C * 250.0)))
        return msg

    def unpack(self, msg):
        (self.index,
         self.millis,
         self.ax_raw,
         self.ay_raw,
         self.az_raw,
         self.hx_raw,
         self.hy_raw,
         self.hz_raw,
         self.ax_mps2,
         self.ay_mps2,
         self.az_mps2,
         self.p_rps,
         self.q_rps,
         self.r_rps,
         self.hx,
         self.hy,
         self.hz,
         self.temp_C) = self._struct.unpack(msg)
        self.ax_raw /= 835.296217
        self.ay_raw /= 835.296217
        self.az_raw /= 835.296217
        self.hx_raw /= 30000.0
        self.hy_raw /= 30000.0
        self.hz_raw /= 30000.0
        self.ax_mps2 /= 835.296217
        self.ay_mps2 /= 835.296217
        self.az_mps2 /= 835.296217
        self.p_rps /= 3754.82165
        self.q_rps /= 3754.82165
        self.r_rps /= 3754.82165
        self.hx /= 30000.0
        self.hy /= 30000.0
        self.hz /= 30000.0
        self.temp_C /= 250.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setUInt("millis", self.millis)
        node.setDouble("ax_raw", self.ax_raw)
        node.setDouble("ay_raw", self.ay_raw)
        node.setDouble("az_raw", self.az_raw)
        node.setDouble("hx_raw", self.hx_raw)
        node.setDouble("hy_raw", self.hy_raw)
        node.setDouble("hz_raw", self.hz_raw)
        node.setDouble("ax_mps2", self.ax_mps2)
        node.setDouble("ay_mps2", self.ay_mps2)
        node.setDouble("az_mps2", self.az_mps2)
        node.setDouble("p_rps", self.p_rps)
        node.setDouble("q_rps", self.q_rps)
        node.setDouble("r_rps", self.r_rps)
        node.setDouble("hx", self.hx)
        node.setDouble("hy", self.hy)
        node.setDouble("hz", self.hz)
        node.setDouble("temp_C", self.temp_C)

    def props2msg(self, node):
        index = node.getUInt("index")
        millis = node.getUInt("millis")
        ax_raw = node.getDouble("ax_raw")
        ay_raw = node.getDouble("ay_raw")
        az_raw = node.getDouble("az_raw")
        hx_raw = node.getDouble("hx_raw")
        hy_raw = node.getDouble("hy_raw")
        hz_raw = node.getDouble("hz_raw")
        ax_mps2 = node.getDouble("ax_mps2")
        ay_mps2 = node.getDouble("ay_mps2")
        az_mps2 = node.getDouble("az_mps2")
        p_rps = node.getDouble("p_rps")
        q_rps = node.getDouble("q_rps")
        r_rps = node.getDouble("r_rps")
        hx = node.getDouble("hx")
        hy = node.getDouble("hy")
        hz = node.getDouble("hz")
        temp_C = node.getDouble("temp_C")

# Message: airdata_v5
# Id: 18
class airdata_v5():
    id = 18
    _pack_string = "<BdHhhffhHBBB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.pressure_mbar = 0.0
        self.temp_C = 0.0
        self.airspeed_smoothed_kt = 0.0
        self.altitude_smoothed_m = 0.0
        self.altitude_true_m = 0.0
        self.pressure_vertical_speed_fps = 0.0
        self.wind_dir_deg = 0.0
        self.wind_speed_kt = 0.0
        self.pitot_scale_factor = 0.0
        self.status = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  int(round(self.pressure_mbar * 10.0)),
                  int(round(self.temp_C * 100.0)),
                  int(round(self.airspeed_smoothed_kt * 100.0)),
                  self.altitude_smoothed_m,
                  self.altitude_true_m,
                  int(round(self.pressure_vertical_speed_fps * 600.0)),
                  int(round(self.wind_dir_deg * 100.0)),
                  int(round(self.wind_speed_kt * 4.0)),
                  int(round(self.pitot_scale_factor * 100.0)),
                  self.status)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.pressure_mbar,
         self.temp_C,
         self.airspeed_smoothed_kt,
         self.altitude_smoothed_m,
         self.altitude_true_m,
         self.pressure_vertical_speed_fps,
         self.wind_dir_deg,
         self.wind_speed_kt,
         self.pitot_scale_factor,
         self.status) = self._struct.unpack(msg)
        self.pressure_mbar /= 10.0
        self.temp_C /= 100.0
        self.airspeed_smoothed_kt /= 100.0
        self.pressure_vertical_speed_fps /= 600.0
        self.wind_dir_deg /= 100.0
        self.wind_speed_kt /= 4.0
        self.pitot_scale_factor /= 100.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("pressure_mbar", self.pressure_mbar)
        node.setDouble("temp_C", self.temp_C)
        node.setDouble("airspeed_smoothed_kt", self.airspeed_smoothed_kt)
        node.setDouble("altitude_smoothed_m", self.altitude_smoothed_m)
        node.setDouble("altitude_true_m", self.altitude_true_m)
        node.setDouble("pressure_vertical_speed_fps", self.pressure_vertical_speed_fps)
        node.setDouble("wind_dir_deg", self.wind_dir_deg)
        node.setDouble("wind_speed_kt", self.wind_speed_kt)
        node.setDouble("pitot_scale_factor", self.pitot_scale_factor)
        node.setUInt("status", self.status)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        pressure_mbar = node.getDouble("pressure_mbar")
        temp_C = node.getDouble("temp_C")
        airspeed_smoothed_kt = node.getDouble("airspeed_smoothed_kt")
        altitude_smoothed_m = node.getDouble("altitude_smoothed_m")
        altitude_true_m = node.getDouble("altitude_true_m")
        pressure_vertical_speed_fps = node.getDouble("pressure_vertical_speed_fps")
        wind_dir_deg = node.getDouble("wind_dir_deg")
        wind_speed_kt = node.getDouble("wind_speed_kt")
        pitot_scale_factor = node.getDouble("pitot_scale_factor")
        status = node.getUInt("status")

# Message: airdata_v6
# Id: 40
class airdata_v6():
    id = 40
    _pack_string = "<BfHhhffhHBBB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.pressure_mbar = 0.0
        self.temp_C = 0.0
        self.airspeed_smoothed_kt = 0.0
        self.altitude_smoothed_m = 0.0
        self.altitude_true_m = 0.0
        self.pressure_vertical_speed_fps = 0.0
        self.wind_dir_deg = 0.0
        self.wind_speed_kt = 0.0
        self.pitot_scale_factor = 0.0
        self.status = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  int(round(self.pressure_mbar * 10.0)),
                  int(round(self.temp_C * 100.0)),
                  int(round(self.airspeed_smoothed_kt * 100.0)),
                  self.altitude_smoothed_m,
                  self.altitude_true_m,
                  int(round(self.pressure_vertical_speed_fps * 600.0)),
                  int(round(self.wind_dir_deg * 100.0)),
                  int(round(self.wind_speed_kt * 4.0)),
                  int(round(self.pitot_scale_factor * 100.0)),
                  self.status)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.pressure_mbar,
         self.temp_C,
         self.airspeed_smoothed_kt,
         self.altitude_smoothed_m,
         self.altitude_true_m,
         self.pressure_vertical_speed_fps,
         self.wind_dir_deg,
         self.wind_speed_kt,
         self.pitot_scale_factor,
         self.status) = self._struct.unpack(msg)
        self.pressure_mbar /= 10.0
        self.temp_C /= 100.0
        self.airspeed_smoothed_kt /= 100.0
        self.pressure_vertical_speed_fps /= 600.0
        self.wind_dir_deg /= 100.0
        self.wind_speed_kt /= 4.0
        self.pitot_scale_factor /= 100.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("pressure_mbar", self.pressure_mbar)
        node.setDouble("temp_C", self.temp_C)
        node.setDouble("airspeed_smoothed_kt", self.airspeed_smoothed_kt)
        node.setDouble("altitude_smoothed_m", self.altitude_smoothed_m)
        node.setDouble("altitude_true_m", self.altitude_true_m)
        node.setDouble("pressure_vertical_speed_fps", self.pressure_vertical_speed_fps)
        node.setDouble("wind_dir_deg", self.wind_dir_deg)
        node.setDouble("wind_speed_kt", self.wind_speed_kt)
        node.setDouble("pitot_scale_factor", self.pitot_scale_factor)
        node.setUInt("status", self.status)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        pressure_mbar = node.getDouble("pressure_mbar")
        temp_C = node.getDouble("temp_C")
        airspeed_smoothed_kt = node.getDouble("airspeed_smoothed_kt")
        altitude_smoothed_m = node.getDouble("altitude_smoothed_m")
        altitude_true_m = node.getDouble("altitude_true_m")
        pressure_vertical_speed_fps = node.getDouble("pressure_vertical_speed_fps")
        wind_dir_deg = node.getDouble("wind_dir_deg")
        wind_speed_kt = node.getDouble("wind_speed_kt")
        pitot_scale_factor = node.getDouble("pitot_scale_factor")
        status = node.getUInt("status")

# Message: airdata_v7
# Id: 43
class airdata_v7():
    id = 43
    _pack_string = "<BfHhhffhHBBHB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.pressure_mbar = 0.0
        self.temp_C = 0.0
        self.airspeed_smoothed_kt = 0.0
        self.altitude_smoothed_m = 0.0
        self.altitude_true_m = 0.0
        self.pressure_vertical_speed_fps = 0.0
        self.wind_dir_deg = 0.0
        self.wind_speed_kt = 0.0
        self.pitot_scale_factor = 0.0
        self.error_count = 0
        self.status = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  int(round(self.pressure_mbar * 10.0)),
                  int(round(self.temp_C * 100.0)),
                  int(round(self.airspeed_smoothed_kt * 100.0)),
                  self.altitude_smoothed_m,
                  self.altitude_true_m,
                  int(round(self.pressure_vertical_speed_fps * 600.0)),
                  int(round(self.wind_dir_deg * 100.0)),
                  int(round(self.wind_speed_kt * 4.0)),
                  int(round(self.pitot_scale_factor * 100.0)),
                  self.error_count,
                  self.status)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.pressure_mbar,
         self.temp_C,
         self.airspeed_smoothed_kt,
         self.altitude_smoothed_m,
         self.altitude_true_m,
         self.pressure_vertical_speed_fps,
         self.wind_dir_deg,
         self.wind_speed_kt,
         self.pitot_scale_factor,
         self.error_count,
         self.status) = self._struct.unpack(msg)
        self.pressure_mbar /= 10.0
        self.temp_C /= 100.0
        self.airspeed_smoothed_kt /= 100.0
        self.pressure_vertical_speed_fps /= 600.0
        self.wind_dir_deg /= 100.0
        self.wind_speed_kt /= 4.0
        self.pitot_scale_factor /= 100.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("pressure_mbar", self.pressure_mbar)
        node.setDouble("temp_C", self.temp_C)
        node.setDouble("airspeed_smoothed_kt", self.airspeed_smoothed_kt)
        node.setDouble("altitude_smoothed_m", self.altitude_smoothed_m)
        node.setDouble("altitude_true_m", self.altitude_true_m)
        node.setDouble("pressure_vertical_speed_fps", self.pressure_vertical_speed_fps)
        node.setDouble("wind_dir_deg", self.wind_dir_deg)
        node.setDouble("wind_speed_kt", self.wind_speed_kt)
        node.setDouble("pitot_scale_factor", self.pitot_scale_factor)
        node.setUInt("error_count", self.error_count)
        node.setUInt("status", self.status)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        pressure_mbar = node.getDouble("pressure_mbar")
        temp_C = node.getDouble("temp_C")
        airspeed_smoothed_kt = node.getDouble("airspeed_smoothed_kt")
        altitude_smoothed_m = node.getDouble("altitude_smoothed_m")
        altitude_true_m = node.getDouble("altitude_true_m")
        pressure_vertical_speed_fps = node.getDouble("pressure_vertical_speed_fps")
        wind_dir_deg = node.getDouble("wind_dir_deg")
        wind_speed_kt = node.getDouble("wind_speed_kt")
        pitot_scale_factor = node.getDouble("pitot_scale_factor")
        error_count = node.getUInt("error_count")
        status = node.getUInt("status")

# Message: filter_v3
# Id: 31
class filter_v3():
    id = 31
    _pack_string = "<BdddfhhhhhhhhhhhhBB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.latitude_deg = 0.0
        self.longitude_deg = 0.0
        self.altitude_m = 0.0
        self.vn_ms = 0.0
        self.ve_ms = 0.0
        self.vd_ms = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.p_bias = 0.0
        self.q_bias = 0.0
        self.r_bias = 0.0
        self.ax_bias = 0.0
        self.ay_bias = 0.0
        self.az_bias = 0.0
        self.sequence_num = 0
        self.status = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  self.latitude_deg,
                  self.longitude_deg,
                  self.altitude_m,
                  int(round(self.vn_ms * 100.0)),
                  int(round(self.ve_ms * 100.0)),
                  int(round(self.vd_ms * 100.0)),
                  int(round(self.roll_deg * 10.0)),
                  int(round(self.pitch_deg * 10.0)),
                  int(round(self.yaw_deg * 10.0)),
                  int(round(self.p_bias * 10000.0)),
                  int(round(self.q_bias * 10000.0)),
                  int(round(self.r_bias * 10000.0)),
                  int(round(self.ax_bias * 1000.0)),
                  int(round(self.ay_bias * 1000.0)),
                  int(round(self.az_bias * 1000.0)),
                  self.sequence_num,
                  self.status)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.latitude_deg,
         self.longitude_deg,
         self.altitude_m,
         self.vn_ms,
         self.ve_ms,
         self.vd_ms,
         self.roll_deg,
         self.pitch_deg,
         self.yaw_deg,
         self.p_bias,
         self.q_bias,
         self.r_bias,
         self.ax_bias,
         self.ay_bias,
         self.az_bias,
         self.sequence_num,
         self.status) = self._struct.unpack(msg)
        self.vn_ms /= 100.0
        self.ve_ms /= 100.0
        self.vd_ms /= 100.0
        self.roll_deg /= 10.0
        self.pitch_deg /= 10.0
        self.yaw_deg /= 10.0
        self.p_bias /= 10000.0
        self.q_bias /= 10000.0
        self.r_bias /= 10000.0
        self.ax_bias /= 1000.0
        self.ay_bias /= 1000.0
        self.az_bias /= 1000.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("latitude_deg", self.latitude_deg)
        node.setDouble("longitude_deg", self.longitude_deg)
        node.setDouble("altitude_m", self.altitude_m)
        node.setDouble("vn_ms", self.vn_ms)
        node.setDouble("ve_ms", self.ve_ms)
        node.setDouble("vd_ms", self.vd_ms)
        node.setDouble("roll_deg", self.roll_deg)
        node.setDouble("pitch_deg", self.pitch_deg)
        node.setDouble("yaw_deg", self.yaw_deg)
        node.setDouble("p_bias", self.p_bias)
        node.setDouble("q_bias", self.q_bias)
        node.setDouble("r_bias", self.r_bias)
        node.setDouble("ax_bias", self.ax_bias)
        node.setDouble("ay_bias", self.ay_bias)
        node.setDouble("az_bias", self.az_bias)
        node.setUInt("sequence_num", self.sequence_num)
        node.setUInt("status", self.status)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        latitude_deg = node.getDouble("latitude_deg")
        longitude_deg = node.getDouble("longitude_deg")
        altitude_m = node.getDouble("altitude_m")
        vn_ms = node.getDouble("vn_ms")
        ve_ms = node.getDouble("ve_ms")
        vd_ms = node.getDouble("vd_ms")
        roll_deg = node.getDouble("roll_deg")
        pitch_deg = node.getDouble("pitch_deg")
        yaw_deg = node.getDouble("yaw_deg")
        p_bias = node.getDouble("p_bias")
        q_bias = node.getDouble("q_bias")
        r_bias = node.getDouble("r_bias")
        ax_bias = node.getDouble("ax_bias")
        ay_bias = node.getDouble("ay_bias")
        az_bias = node.getDouble("az_bias")
        sequence_num = node.getUInt("sequence_num")
        status = node.getUInt("status")

# Message: filter_v4
# Id: 36
class filter_v4():
    id = 36
    _pack_string = "<BfddfhhhhhhhhhhhhBB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.latitude_deg = 0.0
        self.longitude_deg = 0.0
        self.altitude_m = 0.0
        self.vn_ms = 0.0
        self.ve_ms = 0.0
        self.vd_ms = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.p_bias = 0.0
        self.q_bias = 0.0
        self.r_bias = 0.0
        self.ax_bias = 0.0
        self.ay_bias = 0.0
        self.az_bias = 0.0
        self.sequence_num = 0
        self.status = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  self.latitude_deg,
                  self.longitude_deg,
                  self.altitude_m,
                  int(round(self.vn_ms * 100.0)),
                  int(round(self.ve_ms * 100.0)),
                  int(round(self.vd_ms * 100.0)),
                  int(round(self.roll_deg * 10.0)),
                  int(round(self.pitch_deg * 10.0)),
                  int(round(self.yaw_deg * 10.0)),
                  int(round(self.p_bias * 10000.0)),
                  int(round(self.q_bias * 10000.0)),
                  int(round(self.r_bias * 10000.0)),
                  int(round(self.ax_bias * 1000.0)),
                  int(round(self.ay_bias * 1000.0)),
                  int(round(self.az_bias * 1000.0)),
                  self.sequence_num,
                  self.status)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.latitude_deg,
         self.longitude_deg,
         self.altitude_m,
         self.vn_ms,
         self.ve_ms,
         self.vd_ms,
         self.roll_deg,
         self.pitch_deg,
         self.yaw_deg,
         self.p_bias,
         self.q_bias,
         self.r_bias,
         self.ax_bias,
         self.ay_bias,
         self.az_bias,
         self.sequence_num,
         self.status) = self._struct.unpack(msg)
        self.vn_ms /= 100.0
        self.ve_ms /= 100.0
        self.vd_ms /= 100.0
        self.roll_deg /= 10.0
        self.pitch_deg /= 10.0
        self.yaw_deg /= 10.0
        self.p_bias /= 10000.0
        self.q_bias /= 10000.0
        self.r_bias /= 10000.0
        self.ax_bias /= 1000.0
        self.ay_bias /= 1000.0
        self.az_bias /= 1000.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("latitude_deg", self.latitude_deg)
        node.setDouble("longitude_deg", self.longitude_deg)
        node.setDouble("altitude_m", self.altitude_m)
        node.setDouble("vn_ms", self.vn_ms)
        node.setDouble("ve_ms", self.ve_ms)
        node.setDouble("vd_ms", self.vd_ms)
        node.setDouble("roll_deg", self.roll_deg)
        node.setDouble("pitch_deg", self.pitch_deg)
        node.setDouble("yaw_deg", self.yaw_deg)
        node.setDouble("p_bias", self.p_bias)
        node.setDouble("q_bias", self.q_bias)
        node.setDouble("r_bias", self.r_bias)
        node.setDouble("ax_bias", self.ax_bias)
        node.setDouble("ay_bias", self.ay_bias)
        node.setDouble("az_bias", self.az_bias)
        node.setUInt("sequence_num", self.sequence_num)
        node.setUInt("status", self.status)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        latitude_deg = node.getDouble("latitude_deg")
        longitude_deg = node.getDouble("longitude_deg")
        altitude_m = node.getDouble("altitude_m")
        vn_ms = node.getDouble("vn_ms")
        ve_ms = node.getDouble("ve_ms")
        vd_ms = node.getDouble("vd_ms")
        roll_deg = node.getDouble("roll_deg")
        pitch_deg = node.getDouble("pitch_deg")
        yaw_deg = node.getDouble("yaw_deg")
        p_bias = node.getDouble("p_bias")
        q_bias = node.getDouble("q_bias")
        r_bias = node.getDouble("r_bias")
        ax_bias = node.getDouble("ax_bias")
        ay_bias = node.getDouble("ay_bias")
        az_bias = node.getDouble("az_bias")
        sequence_num = node.getUInt("sequence_num")
        status = node.getUInt("status")

# Message: filter_v5
# Id: 47
class filter_v5():
    id = 47
    _pack_string = "<BfddfhhhhhhhhhhhhHHHBB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.latitude_deg = 0.0
        self.longitude_deg = 0.0
        self.altitude_m = 0.0
        self.vn_ms = 0.0
        self.ve_ms = 0.0
        self.vd_ms = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.p_bias = 0.0
        self.q_bias = 0.0
        self.r_bias = 0.0
        self.ax_bias = 0.0
        self.ay_bias = 0.0
        self.az_bias = 0.0
        self.max_pos_cov = 0.0
        self.max_vel_cov = 0.0
        self.max_att_cov = 0.0
        self.sequence_num = 0
        self.status = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  self.latitude_deg,
                  self.longitude_deg,
                  self.altitude_m,
                  int(round(self.vn_ms * 100.0)),
                  int(round(self.ve_ms * 100.0)),
                  int(round(self.vd_ms * 100.0)),
                  int(round(self.roll_deg * 10.0)),
                  int(round(self.pitch_deg * 10.0)),
                  int(round(self.yaw_deg * 10.0)),
                  int(round(self.p_bias * 10000.0)),
                  int(round(self.q_bias * 10000.0)),
                  int(round(self.r_bias * 10000.0)),
                  int(round(self.ax_bias * 1000.0)),
                  int(round(self.ay_bias * 1000.0)),
                  int(round(self.az_bias * 1000.0)),
                  int(round(self.max_pos_cov * 100.0)),
                  int(round(self.max_vel_cov * 1000.0)),
                  int(round(self.max_att_cov * 10000.0)),
                  self.sequence_num,
                  self.status)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.latitude_deg,
         self.longitude_deg,
         self.altitude_m,
         self.vn_ms,
         self.ve_ms,
         self.vd_ms,
         self.roll_deg,
         self.pitch_deg,
         self.yaw_deg,
         self.p_bias,
         self.q_bias,
         self.r_bias,
         self.ax_bias,
         self.ay_bias,
         self.az_bias,
         self.max_pos_cov,
         self.max_vel_cov,
         self.max_att_cov,
         self.sequence_num,
         self.status) = self._struct.unpack(msg)
        self.vn_ms /= 100.0
        self.ve_ms /= 100.0
        self.vd_ms /= 100.0
        self.roll_deg /= 10.0
        self.pitch_deg /= 10.0
        self.yaw_deg /= 10.0
        self.p_bias /= 10000.0
        self.q_bias /= 10000.0
        self.r_bias /= 10000.0
        self.ax_bias /= 1000.0
        self.ay_bias /= 1000.0
        self.az_bias /= 1000.0
        self.max_pos_cov /= 100.0
        self.max_vel_cov /= 1000.0
        self.max_att_cov /= 10000.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("latitude_deg", self.latitude_deg)
        node.setDouble("longitude_deg", self.longitude_deg)
        node.setDouble("altitude_m", self.altitude_m)
        node.setDouble("vn_ms", self.vn_ms)
        node.setDouble("ve_ms", self.ve_ms)
        node.setDouble("vd_ms", self.vd_ms)
        node.setDouble("roll_deg", self.roll_deg)
        node.setDouble("pitch_deg", self.pitch_deg)
        node.setDouble("yaw_deg", self.yaw_deg)
        node.setDouble("p_bias", self.p_bias)
        node.setDouble("q_bias", self.q_bias)
        node.setDouble("r_bias", self.r_bias)
        node.setDouble("ax_bias", self.ax_bias)
        node.setDouble("ay_bias", self.ay_bias)
        node.setDouble("az_bias", self.az_bias)
        node.setDouble("max_pos_cov", self.max_pos_cov)
        node.setDouble("max_vel_cov", self.max_vel_cov)
        node.setDouble("max_att_cov", self.max_att_cov)
        node.setUInt("sequence_num", self.sequence_num)
        node.setUInt("status", self.status)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        latitude_deg = node.getDouble("latitude_deg")
        longitude_deg = node.getDouble("longitude_deg")
        altitude_m = node.getDouble("altitude_m")
        vn_ms = node.getDouble("vn_ms")
        ve_ms = node.getDouble("ve_ms")
        vd_ms = node.getDouble("vd_ms")
        roll_deg = node.getDouble("roll_deg")
        pitch_deg = node.getDouble("pitch_deg")
        yaw_deg = node.getDouble("yaw_deg")
        p_bias = node.getDouble("p_bias")
        q_bias = node.getDouble("q_bias")
        r_bias = node.getDouble("r_bias")
        ax_bias = node.getDouble("ax_bias")
        ay_bias = node.getDouble("ay_bias")
        az_bias = node.getDouble("az_bias")
        max_pos_cov = node.getDouble("max_pos_cov")
        max_vel_cov = node.getDouble("max_vel_cov")
        max_att_cov = node.getDouble("max_att_cov")
        sequence_num = node.getUInt("sequence_num")
        status = node.getUInt("status")

# Message: actuator_v2
# Id: 21
class actuator_v2():
    id = 21
    _pack_string = "<BdhhHhhhhhB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.aileron = 0.0
        self.elevator = 0.0
        self.throttle = 0.0
        self.rudder = 0.0
        self.channel5 = 0.0
        self.flaps = 0.0
        self.channel7 = 0.0
        self.channel8 = 0.0
        self.status = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  int(round(self.aileron * 20000.0)),
                  int(round(self.elevator * 20000.0)),
                  int(round(self.throttle * 60000.0)),
                  int(round(self.rudder * 20000.0)),
                  int(round(self.channel5 * 20000.0)),
                  int(round(self.flaps * 20000.0)),
                  int(round(self.channel7 * 20000.0)),
                  int(round(self.channel8 * 20000.0)),
                  self.status)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.aileron,
         self.elevator,
         self.throttle,
         self.rudder,
         self.channel5,
         self.flaps,
         self.channel7,
         self.channel8,
         self.status) = self._struct.unpack(msg)
        self.aileron /= 20000.0
        self.elevator /= 20000.0
        self.throttle /= 60000.0
        self.rudder /= 20000.0
        self.channel5 /= 20000.0
        self.flaps /= 20000.0
        self.channel7 /= 20000.0
        self.channel8 /= 20000.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("aileron", self.aileron)
        node.setDouble("elevator", self.elevator)
        node.setDouble("throttle", self.throttle)
        node.setDouble("rudder", self.rudder)
        node.setDouble("channel5", self.channel5)
        node.setDouble("flaps", self.flaps)
        node.setDouble("channel7", self.channel7)
        node.setDouble("channel8", self.channel8)
        node.setUInt("status", self.status)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        aileron = node.getDouble("aileron")
        elevator = node.getDouble("elevator")
        throttle = node.getDouble("throttle")
        rudder = node.getDouble("rudder")
        channel5 = node.getDouble("channel5")
        flaps = node.getDouble("flaps")
        channel7 = node.getDouble("channel7")
        channel8 = node.getDouble("channel8")
        status = node.getUInt("status")

# Message: actuator_v3
# Id: 37
class actuator_v3():
    id = 37
    _pack_string = "<BfhhHhhhhhB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.aileron = 0.0
        self.elevator = 0.0
        self.throttle = 0.0
        self.rudder = 0.0
        self.channel5 = 0.0
        self.flaps = 0.0
        self.channel7 = 0.0
        self.channel8 = 0.0
        self.status = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  int(round(self.aileron * 20000.0)),
                  int(round(self.elevator * 20000.0)),
                  int(round(self.throttle * 60000.0)),
                  int(round(self.rudder * 20000.0)),
                  int(round(self.channel5 * 20000.0)),
                  int(round(self.flaps * 20000.0)),
                  int(round(self.channel7 * 20000.0)),
                  int(round(self.channel8 * 20000.0)),
                  self.status)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.aileron,
         self.elevator,
         self.throttle,
         self.rudder,
         self.channel5,
         self.flaps,
         self.channel7,
         self.channel8,
         self.status) = self._struct.unpack(msg)
        self.aileron /= 20000.0
        self.elevator /= 20000.0
        self.throttle /= 60000.0
        self.rudder /= 20000.0
        self.channel5 /= 20000.0
        self.flaps /= 20000.0
        self.channel7 /= 20000.0
        self.channel8 /= 20000.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("aileron", self.aileron)
        node.setDouble("elevator", self.elevator)
        node.setDouble("throttle", self.throttle)
        node.setDouble("rudder", self.rudder)
        node.setDouble("channel5", self.channel5)
        node.setDouble("flaps", self.flaps)
        node.setDouble("channel7", self.channel7)
        node.setDouble("channel8", self.channel8)
        node.setUInt("status", self.status)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        aileron = node.getDouble("aileron")
        elevator = node.getDouble("elevator")
        throttle = node.getDouble("throttle")
        rudder = node.getDouble("rudder")
        channel5 = node.getDouble("channel5")
        flaps = node.getDouble("flaps")
        channel7 = node.getDouble("channel7")
        channel8 = node.getDouble("channel8")
        status = node.getUInt("status")

# Message: pilot_v2
# Id: 20
class pilot_v2():
    id = 20
    _pack_string = "<BdhhhhhhhhB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.channel = [0.0] * 8
        self.status = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  int(round(self.channel[0] * 20000.0)),
                  int(round(self.channel[1] * 20000.0)),
                  int(round(self.channel[2] * 20000.0)),
                  int(round(self.channel[3] * 20000.0)),
                  int(round(self.channel[4] * 20000.0)),
                  int(round(self.channel[5] * 20000.0)),
                  int(round(self.channel[6] * 20000.0)),
                  int(round(self.channel[7] * 20000.0)),
                  self.status)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.channel[0],
         self.channel[1],
         self.channel[2],
         self.channel[3],
         self.channel[4],
         self.channel[5],
         self.channel[6],
         self.channel[7],
         self.status) = self._struct.unpack(msg)
        self.channel[0] /= 20000.0
        self.channel[1] /= 20000.0
        self.channel[2] /= 20000.0
        self.channel[3] /= 20000.0
        self.channel[4] /= 20000.0
        self.channel[5] /= 20000.0
        self.channel[6] /= 20000.0
        self.channel[7] /= 20000.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        for _i in range(8): node.setDouble("channel", self.channel[_i], _i)
        node.setUInt("status", self.status)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        for _i in range(8): channel[_i] = node.getDouble("channel", _i)
        status = node.getUInt("status")

# Message: pilot_v3
# Id: 38
class pilot_v3():
    id = 38
    _pack_string = "<BfhhhhhhhhB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.channel = [0.0] * 8
        self.status = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  int(round(self.channel[0] * 20000.0)),
                  int(round(self.channel[1] * 20000.0)),
                  int(round(self.channel[2] * 20000.0)),
                  int(round(self.channel[3] * 20000.0)),
                  int(round(self.channel[4] * 20000.0)),
                  int(round(self.channel[5] * 20000.0)),
                  int(round(self.channel[6] * 20000.0)),
                  int(round(self.channel[7] * 20000.0)),
                  self.status)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.channel[0],
         self.channel[1],
         self.channel[2],
         self.channel[3],
         self.channel[4],
         self.channel[5],
         self.channel[6],
         self.channel[7],
         self.status) = self._struct.unpack(msg)
        self.channel[0] /= 20000.0
        self.channel[1] /= 20000.0
        self.channel[2] /= 20000.0
        self.channel[3] /= 20000.0
        self.channel[4] /= 20000.0
        self.channel[5] /= 20000.0
        self.channel[6] /= 20000.0
        self.channel[7] /= 20000.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        for _i in range(8): node.setDouble("channel", self.channel[_i], _i)
        node.setUInt("status", self.status)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        for _i in range(8): channel[_i] = node.getDouble("channel", _i)
        status = node.getUInt("status")

# Message: ap_status_v4
# Id: 30
class ap_status_v4():
    id = 30
    _pack_string = "<BdhhHHhhHHddHHB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.groundtrack_deg = 0.0
        self.roll_deg = 0.0
        self.altitude_msl_ft = 0
        self.altitude_ground_m = 0
        self.pitch_deg = 0.0
        self.airspeed_kt = 0.0
        self.flight_timer = 0
        self.target_waypoint_idx = 0
        self.wp_longitude_deg = 0.0
        self.wp_latitude_deg = 0.0
        self.wp_index = 0
        self.route_size = 0
        self.sequence_num = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  int(round(self.groundtrack_deg * 10.0)),
                  int(round(self.roll_deg * 10.0)),
                  self.altitude_msl_ft,
                  self.altitude_ground_m,
                  int(round(self.pitch_deg * 10.0)),
                  int(round(self.airspeed_kt * 10.0)),
                  self.flight_timer,
                  self.target_waypoint_idx,
                  self.wp_longitude_deg,
                  self.wp_latitude_deg,
                  self.wp_index,
                  self.route_size,
                  self.sequence_num)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.groundtrack_deg,
         self.roll_deg,
         self.altitude_msl_ft,
         self.altitude_ground_m,
         self.pitch_deg,
         self.airspeed_kt,
         self.flight_timer,
         self.target_waypoint_idx,
         self.wp_longitude_deg,
         self.wp_latitude_deg,
         self.wp_index,
         self.route_size,
         self.sequence_num) = self._struct.unpack(msg)
        self.groundtrack_deg /= 10.0
        self.roll_deg /= 10.0
        self.pitch_deg /= 10.0
        self.airspeed_kt /= 10.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("groundtrack_deg", self.groundtrack_deg)
        node.setDouble("roll_deg", self.roll_deg)
        node.setUInt("altitude_msl_ft", self.altitude_msl_ft)
        node.setUInt("altitude_ground_m", self.altitude_ground_m)
        node.setDouble("pitch_deg", self.pitch_deg)
        node.setDouble("airspeed_kt", self.airspeed_kt)
        node.setUInt("flight_timer", self.flight_timer)
        node.setUInt("target_waypoint_idx", self.target_waypoint_idx)
        node.setDouble("wp_longitude_deg", self.wp_longitude_deg)
        node.setDouble("wp_latitude_deg", self.wp_latitude_deg)
        node.setUInt("wp_index", self.wp_index)
        node.setUInt("route_size", self.route_size)
        node.setUInt("sequence_num", self.sequence_num)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        groundtrack_deg = node.getDouble("groundtrack_deg")
        roll_deg = node.getDouble("roll_deg")
        altitude_msl_ft = node.getUInt("altitude_msl_ft")
        altitude_ground_m = node.getUInt("altitude_ground_m")
        pitch_deg = node.getDouble("pitch_deg")
        airspeed_kt = node.getDouble("airspeed_kt")
        flight_timer = node.getUInt("flight_timer")
        target_waypoint_idx = node.getUInt("target_waypoint_idx")
        wp_longitude_deg = node.getDouble("wp_longitude_deg")
        wp_latitude_deg = node.getDouble("wp_latitude_deg")
        wp_index = node.getUInt("wp_index")
        route_size = node.getUInt("route_size")
        sequence_num = node.getUInt("sequence_num")

# Message: ap_status_v5
# Id: 32
class ap_status_v5():
    id = 32
    _pack_string = "<BdBhhHHhhHHddHHB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.flags = 0
        self.groundtrack_deg = 0.0
        self.roll_deg = 0.0
        self.altitude_msl_ft = 0
        self.altitude_ground_m = 0
        self.pitch_deg = 0.0
        self.airspeed_kt = 0.0
        self.flight_timer = 0
        self.target_waypoint_idx = 0
        self.wp_longitude_deg = 0.0
        self.wp_latitude_deg = 0.0
        self.wp_index = 0
        self.route_size = 0
        self.sequence_num = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  self.flags,
                  int(round(self.groundtrack_deg * 10.0)),
                  int(round(self.roll_deg * 10.0)),
                  self.altitude_msl_ft,
                  self.altitude_ground_m,
                  int(round(self.pitch_deg * 10.0)),
                  int(round(self.airspeed_kt * 10.0)),
                  self.flight_timer,
                  self.target_waypoint_idx,
                  self.wp_longitude_deg,
                  self.wp_latitude_deg,
                  self.wp_index,
                  self.route_size,
                  self.sequence_num)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.flags,
         self.groundtrack_deg,
         self.roll_deg,
         self.altitude_msl_ft,
         self.altitude_ground_m,
         self.pitch_deg,
         self.airspeed_kt,
         self.flight_timer,
         self.target_waypoint_idx,
         self.wp_longitude_deg,
         self.wp_latitude_deg,
         self.wp_index,
         self.route_size,
         self.sequence_num) = self._struct.unpack(msg)
        self.groundtrack_deg /= 10.0
        self.roll_deg /= 10.0
        self.pitch_deg /= 10.0
        self.airspeed_kt /= 10.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setUInt("flags", self.flags)
        node.setDouble("groundtrack_deg", self.groundtrack_deg)
        node.setDouble("roll_deg", self.roll_deg)
        node.setUInt("altitude_msl_ft", self.altitude_msl_ft)
        node.setUInt("altitude_ground_m", self.altitude_ground_m)
        node.setDouble("pitch_deg", self.pitch_deg)
        node.setDouble("airspeed_kt", self.airspeed_kt)
        node.setUInt("flight_timer", self.flight_timer)
        node.setUInt("target_waypoint_idx", self.target_waypoint_idx)
        node.setDouble("wp_longitude_deg", self.wp_longitude_deg)
        node.setDouble("wp_latitude_deg", self.wp_latitude_deg)
        node.setUInt("wp_index", self.wp_index)
        node.setUInt("route_size", self.route_size)
        node.setUInt("sequence_num", self.sequence_num)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        flags = node.getUInt("flags")
        groundtrack_deg = node.getDouble("groundtrack_deg")
        roll_deg = node.getDouble("roll_deg")
        altitude_msl_ft = node.getUInt("altitude_msl_ft")
        altitude_ground_m = node.getUInt("altitude_ground_m")
        pitch_deg = node.getDouble("pitch_deg")
        airspeed_kt = node.getDouble("airspeed_kt")
        flight_timer = node.getUInt("flight_timer")
        target_waypoint_idx = node.getUInt("target_waypoint_idx")
        wp_longitude_deg = node.getDouble("wp_longitude_deg")
        wp_latitude_deg = node.getDouble("wp_latitude_deg")
        wp_index = node.getUInt("wp_index")
        route_size = node.getUInt("route_size")
        sequence_num = node.getUInt("sequence_num")

# Message: ap_status_v6
# Id: 33
class ap_status_v6():
    id = 33
    _pack_string = "<BdBhhHHhhHHddHHBHB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.flags = 0
        self.groundtrack_deg = 0.0
        self.roll_deg = 0.0
        self.altitude_msl_ft = 0
        self.altitude_ground_m = 0
        self.pitch_deg = 0.0
        self.airspeed_kt = 0.0
        self.flight_timer = 0
        self.target_waypoint_idx = 0
        self.wp_longitude_deg = 0.0
        self.wp_latitude_deg = 0.0
        self.wp_index = 0
        self.route_size = 0
        self.task_id = 0
        self.task_attribute = 0
        self.sequence_num = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  self.flags,
                  int(round(self.groundtrack_deg * 10.0)),
                  int(round(self.roll_deg * 10.0)),
                  self.altitude_msl_ft,
                  self.altitude_ground_m,
                  int(round(self.pitch_deg * 10.0)),
                  int(round(self.airspeed_kt * 10.0)),
                  self.flight_timer,
                  self.target_waypoint_idx,
                  self.wp_longitude_deg,
                  self.wp_latitude_deg,
                  self.wp_index,
                  self.route_size,
                  self.task_id,
                  self.task_attribute,
                  self.sequence_num)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.flags,
         self.groundtrack_deg,
         self.roll_deg,
         self.altitude_msl_ft,
         self.altitude_ground_m,
         self.pitch_deg,
         self.airspeed_kt,
         self.flight_timer,
         self.target_waypoint_idx,
         self.wp_longitude_deg,
         self.wp_latitude_deg,
         self.wp_index,
         self.route_size,
         self.task_id,
         self.task_attribute,
         self.sequence_num) = self._struct.unpack(msg)
        self.groundtrack_deg /= 10.0
        self.roll_deg /= 10.0
        self.pitch_deg /= 10.0
        self.airspeed_kt /= 10.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setUInt("flags", self.flags)
        node.setDouble("groundtrack_deg", self.groundtrack_deg)
        node.setDouble("roll_deg", self.roll_deg)
        node.setUInt("altitude_msl_ft", self.altitude_msl_ft)
        node.setUInt("altitude_ground_m", self.altitude_ground_m)
        node.setDouble("pitch_deg", self.pitch_deg)
        node.setDouble("airspeed_kt", self.airspeed_kt)
        node.setUInt("flight_timer", self.flight_timer)
        node.setUInt("target_waypoint_idx", self.target_waypoint_idx)
        node.setDouble("wp_longitude_deg", self.wp_longitude_deg)
        node.setDouble("wp_latitude_deg", self.wp_latitude_deg)
        node.setUInt("wp_index", self.wp_index)
        node.setUInt("route_size", self.route_size)
        node.setUInt("task_id", self.task_id)
        node.setUInt("task_attribute", self.task_attribute)
        node.setUInt("sequence_num", self.sequence_num)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        flags = node.getUInt("flags")
        groundtrack_deg = node.getDouble("groundtrack_deg")
        roll_deg = node.getDouble("roll_deg")
        altitude_msl_ft = node.getUInt("altitude_msl_ft")
        altitude_ground_m = node.getUInt("altitude_ground_m")
        pitch_deg = node.getDouble("pitch_deg")
        airspeed_kt = node.getDouble("airspeed_kt")
        flight_timer = node.getUInt("flight_timer")
        target_waypoint_idx = node.getUInt("target_waypoint_idx")
        wp_longitude_deg = node.getDouble("wp_longitude_deg")
        wp_latitude_deg = node.getDouble("wp_latitude_deg")
        wp_index = node.getUInt("wp_index")
        route_size = node.getUInt("route_size")
        task_id = node.getUInt("task_id")
        task_attribute = node.getUInt("task_attribute")
        sequence_num = node.getUInt("sequence_num")

# Message: ap_status_v7
# Id: 39
class ap_status_v7():
    id = 39
    _pack_string = "<BfBhhHHhhHHddHHBHB"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.flags = 0
        self.groundtrack_deg = 0.0
        self.roll_deg = 0.0
        self.altitude_msl_ft = 0.0
        self.altitude_ground_m = 0.0
        self.pitch_deg = 0.0
        self.airspeed_kt = 0.0
        self.flight_timer = 0.0
        self.target_waypoint_idx = 0
        self.wp_longitude_deg = 0.0
        self.wp_latitude_deg = 0.0
        self.wp_index = 0
        self.route_size = 0
        self.task_id = 0
        self.task_attribute = 0
        self.sequence_num = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  self.flags,
                  int(round(self.groundtrack_deg * 10.0)),
                  int(round(self.roll_deg * 10.0)),
                  int(round(self.altitude_msl_ft * 1.0)),
                  int(round(self.altitude_ground_m * 1.0)),
                  int(round(self.pitch_deg * 10.0)),
                  int(round(self.airspeed_kt * 10.0)),
                  int(round(self.flight_timer * 1.0)),
                  self.target_waypoint_idx,
                  self.wp_longitude_deg,
                  self.wp_latitude_deg,
                  self.wp_index,
                  self.route_size,
                  self.task_id,
                  self.task_attribute,
                  self.sequence_num)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.flags,
         self.groundtrack_deg,
         self.roll_deg,
         self.altitude_msl_ft,
         self.altitude_ground_m,
         self.pitch_deg,
         self.airspeed_kt,
         self.flight_timer,
         self.target_waypoint_idx,
         self.wp_longitude_deg,
         self.wp_latitude_deg,
         self.wp_index,
         self.route_size,
         self.task_id,
         self.task_attribute,
         self.sequence_num) = self._struct.unpack(msg)
        self.groundtrack_deg /= 10.0
        self.roll_deg /= 10.0
        self.altitude_msl_ft /= 1.0
        self.altitude_ground_m /= 1.0
        self.pitch_deg /= 10.0
        self.airspeed_kt /= 10.0
        self.flight_timer /= 1.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setUInt("flags", self.flags)
        node.setDouble("groundtrack_deg", self.groundtrack_deg)
        node.setDouble("roll_deg", self.roll_deg)
        node.setDouble("altitude_msl_ft", self.altitude_msl_ft)
        node.setDouble("altitude_ground_m", self.altitude_ground_m)
        node.setDouble("pitch_deg", self.pitch_deg)
        node.setDouble("airspeed_kt", self.airspeed_kt)
        node.setDouble("flight_timer", self.flight_timer)
        node.setUInt("target_waypoint_idx", self.target_waypoint_idx)
        node.setDouble("wp_longitude_deg", self.wp_longitude_deg)
        node.setDouble("wp_latitude_deg", self.wp_latitude_deg)
        node.setUInt("wp_index", self.wp_index)
        node.setUInt("route_size", self.route_size)
        node.setUInt("task_id", self.task_id)
        node.setUInt("task_attribute", self.task_attribute)
        node.setUInt("sequence_num", self.sequence_num)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        flags = node.getUInt("flags")
        groundtrack_deg = node.getDouble("groundtrack_deg")
        roll_deg = node.getDouble("roll_deg")
        altitude_msl_ft = node.getDouble("altitude_msl_ft")
        altitude_ground_m = node.getDouble("altitude_ground_m")
        pitch_deg = node.getDouble("pitch_deg")
        airspeed_kt = node.getDouble("airspeed_kt")
        flight_timer = node.getDouble("flight_timer")
        target_waypoint_idx = node.getUInt("target_waypoint_idx")
        wp_longitude_deg = node.getDouble("wp_longitude_deg")
        wp_latitude_deg = node.getDouble("wp_latitude_deg")
        wp_index = node.getUInt("wp_index")
        route_size = node.getUInt("route_size")
        task_id = node.getUInt("task_id")
        task_attribute = node.getUInt("task_attribute")
        sequence_num = node.getUInt("sequence_num")

# Message: system_health_v4
# Id: 19
class system_health_v4():
    id = 19
    _pack_string = "<BdHHHHHH"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.system_load_avg = 0.0
        self.avionics_vcc = 0.0
        self.main_vcc = 0.0
        self.cell_vcc = 0.0
        self.main_amps = 0.0
        self.total_mah = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  int(round(self.system_load_avg * 100.0)),
                  int(round(self.avionics_vcc * 1000.0)),
                  int(round(self.main_vcc * 1000.0)),
                  int(round(self.cell_vcc * 1000.0)),
                  int(round(self.main_amps * 1000.0)),
                  int(round(self.total_mah * 10.0)))
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.system_load_avg,
         self.avionics_vcc,
         self.main_vcc,
         self.cell_vcc,
         self.main_amps,
         self.total_mah) = self._struct.unpack(msg)
        self.system_load_avg /= 100.0
        self.avionics_vcc /= 1000.0
        self.main_vcc /= 1000.0
        self.cell_vcc /= 1000.0
        self.main_amps /= 1000.0
        self.total_mah /= 10.0

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("system_load_avg", self.system_load_avg)
        node.setDouble("avionics_vcc", self.avionics_vcc)
        node.setDouble("main_vcc", self.main_vcc)
        node.setDouble("cell_vcc", self.cell_vcc)
        node.setDouble("main_amps", self.main_amps)
        node.setDouble("total_mah", self.total_mah)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        system_load_avg = node.getDouble("system_load_avg")
        avionics_vcc = node.getDouble("avionics_vcc")
        main_vcc = node.getDouble("main_vcc")
        cell_vcc = node.getDouble("cell_vcc")
        main_amps = node.getDouble("main_amps")
        total_mah = node.getDouble("total_mah")

# Message: system_health_v5
# Id: 41
class system_health_v5():
    id = 41
    _pack_string = "<BfHHHHHH"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.system_load_avg = 0.0
        self.avionics_vcc = 0.0
        self.main_vcc = 0.0
        self.cell_vcc = 0.0
        self.main_amps = 0.0
        self.total_mah = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  int(round(self.system_load_avg * 100.0)),
                  int(round(self.avionics_vcc * 1000.0)),
                  int(round(self.main_vcc * 1000.0)),
                  int(round(self.cell_vcc * 1000.0)),
                  int(round(self.main_amps * 1000.0)),
                  int(round(self.total_mah * 0.1)))
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.system_load_avg,
         self.avionics_vcc,
         self.main_vcc,
         self.cell_vcc,
         self.main_amps,
         self.total_mah) = self._struct.unpack(msg)
        self.system_load_avg /= 100.0
        self.avionics_vcc /= 1000.0
        self.main_vcc /= 1000.0
        self.cell_vcc /= 1000.0
        self.main_amps /= 1000.0
        self.total_mah /= 0.1

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("system_load_avg", self.system_load_avg)
        node.setDouble("avionics_vcc", self.avionics_vcc)
        node.setDouble("main_vcc", self.main_vcc)
        node.setDouble("cell_vcc", self.cell_vcc)
        node.setDouble("main_amps", self.main_amps)
        node.setDouble("total_mah", self.total_mah)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        system_load_avg = node.getDouble("system_load_avg")
        avionics_vcc = node.getDouble("avionics_vcc")
        main_vcc = node.getDouble("main_vcc")
        cell_vcc = node.getDouble("cell_vcc")
        main_amps = node.getDouble("main_amps")
        total_mah = node.getDouble("total_mah")

# Message: system_health_v6
# Id: 46
class system_health_v6():
    id = 46
    _pack_string = "<BfHHHHHHH"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.system_load_avg = 0.0
        self.fmu_timer_misses = 0
        self.avionics_vcc = 0.0
        self.main_vcc = 0.0
        self.cell_vcc = 0.0
        self.main_amps = 0.0
        self.total_mah = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  int(round(self.system_load_avg * 100.0)),
                  self.fmu_timer_misses,
                  int(round(self.avionics_vcc * 1000.0)),
                  int(round(self.main_vcc * 1000.0)),
                  int(round(self.cell_vcc * 1000.0)),
                  int(round(self.main_amps * 1000.0)),
                  int(round(self.total_mah * 0.1)))
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.system_load_avg,
         self.fmu_timer_misses,
         self.avionics_vcc,
         self.main_vcc,
         self.cell_vcc,
         self.main_amps,
         self.total_mah) = self._struct.unpack(msg)
        self.system_load_avg /= 100.0
        self.avionics_vcc /= 1000.0
        self.main_vcc /= 1000.0
        self.cell_vcc /= 1000.0
        self.main_amps /= 1000.0
        self.total_mah /= 0.1

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setDouble("system_load_avg", self.system_load_avg)
        node.setUInt("fmu_timer_misses", self.fmu_timer_misses)
        node.setDouble("avionics_vcc", self.avionics_vcc)
        node.setDouble("main_vcc", self.main_vcc)
        node.setDouble("cell_vcc", self.cell_vcc)
        node.setDouble("main_amps", self.main_amps)
        node.setDouble("total_mah", self.total_mah)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        system_load_avg = node.getDouble("system_load_avg")
        fmu_timer_misses = node.getUInt("fmu_timer_misses")
        avionics_vcc = node.getDouble("avionics_vcc")
        main_vcc = node.getDouble("main_vcc")
        cell_vcc = node.getDouble("cell_vcc")
        main_amps = node.getDouble("main_amps")
        total_mah = node.getDouble("total_mah")

# Message: payload_v2
# Id: 23
class payload_v2():
    id = 23
    _pack_string = "<BdH"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.trigger_num = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  self.trigger_num)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.trigger_num) = self._struct.unpack(msg)

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setUInt("trigger_num", self.trigger_num)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        trigger_num = node.getUInt("trigger_num")

# Message: payload_v3
# Id: 42
class payload_v3():
    id = 42
    _pack_string = "<BfH"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.trigger_num = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  self.trigger_num)
        return msg

    def unpack(self, msg):
        (self.index,
         self.timestamp_sec,
         self.trigger_num) = self._struct.unpack(msg)

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setUInt("trigger_num", self.trigger_num)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        trigger_num = node.getUInt("trigger_num")

# Message: event_v1
# Id: 27
class event_v1():
    id = 27
    _pack_string = "<BdH"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.index = 0
        self.timestamp_sec = 0.0
        self.message = ""
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.index,
                  self.timestamp_sec,
                  len(self.message))
        msg += str.encode(self.message)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        (self.index,
         self.timestamp_sec,
         self.message_len) = self._struct.unpack(msg)
        self.message = extra[:self.message_len].decode()
        extra = extra[self.message_len:]

    def msg2props(self, node):
        node.setUInt("index", self.index)
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setString("message", self.message)

    def props2msg(self, node):
        index = node.getUInt("index")
        timestamp_sec = node.getDouble("timestamp_sec")
        message = node.getString("message")

# Message: event_v2
# Id: 44
class event_v2():
    id = 44
    _pack_string = "<fBH"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.timestamp_sec = 0.0
        self.sequence_num = 0
        self.message = ""
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.timestamp_sec,
                  self.sequence_num,
                  len(self.message))
        msg += str.encode(self.message)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        (self.timestamp_sec,
         self.sequence_num,
         self.message_len) = self._struct.unpack(msg)
        self.message = extra[:self.message_len].decode()
        extra = extra[self.message_len:]

    def msg2props(self, node):
        node.setDouble("timestamp_sec", self.timestamp_sec)
        node.setUInt("sequence_num", self.sequence_num)
        node.setString("message", self.message)

    def props2msg(self, node):
        timestamp_sec = node.getDouble("timestamp_sec")
        sequence_num = node.getUInt("sequence_num")
        message = node.getString("message")

# Message: command_v1
# Id: 28
class command_v1():
    id = 28
    _pack_string = "<BH"
    _struct = struct.Struct(_pack_string)

    def __init__(self, msg=None):
        # public fields
        self.sequence_num = 0
        self.message = ""
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = self._struct.pack(
                  self.sequence_num,
                  len(self.message))
        msg += str.encode(self.message)
        return msg

    def unpack(self, msg):
        base_len = struct.calcsize(self._pack_string)
        extra = msg[base_len:]
        msg = msg[:base_len]
        (self.sequence_num,
         self.message_len) = self._struct.unpack(msg)
        self.message = extra[:self.message_len].decode()
        extra = extra[self.message_len:]

    def msg2props(self, node):
        node.setUInt("sequence_num", self.sequence_num)
        node.setString("message", self.message)

    def props2msg(self, node):
        sequence_num = node.getUInt("sequence_num")
        message = node.getString("message")


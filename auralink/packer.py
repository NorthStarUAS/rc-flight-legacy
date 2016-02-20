import struct

from props import root, getNode

# FIXME: we are hard coding status flag to zero in many places which
# means we aren't using them properly (and/or wasting bytes)

# python struct package notes:
#
# < means use little endian order, no byte alignment
# b = int8_t, B = uint8_t
# h = int16_t, H = uint16_t
# f = float, d = double

gps_node = getNode("/sensors/gps", True)
gps_v1_fmt = '<dddfhhhdBB'
gps_v1_size = struct.calcsize(gps_v1_fmt)

imu_node = getNode("/sensors/imu", True)
imu_v1_fmt = '<dfffffffffB'
imu_v1_size = struct.calcsize(imu_v1_fmt)
imu_v2_fmt = '<dfffffffffhB'
imu_v2_size = struct.calcsize(imu_v2_fmt)

airdata_node = getNode("/sensors/airdata", True)
pos_node = getNode("/position", True)
pos_pressure_node = getNode("/position/pressure", True)
pos_combined_node = getNode("/position/combined", True)
vel_node = getNode("/velocity", True)
wind_node = getNode("/filters/wind", True)
airdata_v1_fmt = "<dhfhhB"
airdata_v1_size = struct.calcsize(airdata_v1_fmt)
airdata_v2_fmt = "<dhfhhHBBB"
airdata_v2_size = struct.calcsize(airdata_v2_fmt)
airdata_v3_fmt = "<dHhhfhhHBBB"
airdata_v3_size = struct.calcsize(airdata_v3_fmt)
airdata_v4_fmt = "<dHhhffhhHBBB"
airdata_v4_size = struct.calcsize(airdata_v4_fmt)

filter_node = getNode("/filters/filter", True)
remote_link_node = getNode("/comms/remote_link", True);
filter_v1_fmt = "<dddfhhhhhhBB"
filter_v1_size = struct.calcsize(filter_v1_fmt)

def pack_gps_v1():
    buf = struct.pack(gps_v1_fmt,
                      gps_node.getFloat("timestamp"),
                      gps_node.getFloat("latitude_deg"),
                      gps_node.getFloat("longitude_deg"),
                      gps_node.getFloat("altitude_m"),
                      int(gps_node.getFloat("vn_ms") * 100),
                      int(gps_node.getFloat("ve_ms") * 100),
                      int(gps_node.getFloat("vd_ms") * 100),
                      gps_node.getFloat("unix_time_sec"),
                      gps_node.getInt("satellites"),
                      0)
    return (buf, gps_v1_size)

def unpack_gps_v1(buf):
    result = struct.unpack(gps_v1_fmt, buf)
    print result
    gps_node.setFloat("timestamp", result[0])
    gps_node.setFloat("latitude_deg", result[1])
    gps_node.setFloat("longitude_deg", result[2])
    gps_node.setFloat("altitude_m", result[3])
    gps_node.setFloat("vn_ms", result[4] / 100.0)
    gps_node.setFloat("ve_ms", result[5] / 100.0)
    gps_node.setFloat("vd_ms", result[6] / 100.0)
    gps_node.setFloat("unix_time_sec", result[7])
    gps_node.setInt("satellites", result[8])
    gps_node.setInt("status", result[9])
    
def pack_imu_v2():
    buf = struct.pack(imu_v2_fmt,
                      imu_node.getFloat("timestamp"),
                      imu_node.getFloat("p_rad_sec"),
                      imu_node.getFloat("q_rad_sec"),
                      imu_node.getFloat("r_rad_sec"),
                      imu_node.getFloat("ax_mps_sec"),
                      imu_node.getFloat("ay_mps_sec"),
                      imu_node.getFloat("az_mps_sec"),
                      imu_node.getFloat("hx"),
                      imu_node.getFloat("hy"),
                      imu_node.getFloat("hz"),
                      int(imu_node.getFloat("temp_C") * 10.0),
                      0)
    return (buf, imu_v2_size)

def unpack_imu_v1(buf):
    result = struct.unpack(imu_v1_fmt, buf)
    print result
    imu_node.setFloat("timestamp", result[0])
    imu_node.setFloat("p_rad_sec", result[1])
    imu_node.setFloat("q_rad_sec", result[2])
    imu_node.setFloat("r_rad_sec", result[3])
    imu_node.setFloat("ax_mps_sec", result[4])
    imu_node.setFloat("ay_mps_sec", result[5])
    imu_node.setFloat("az_mps_sec", result[6])
    imu_node.setFloat("hx", result[7])
    imu_node.setFloat("hy", result[8])
    imu_node.setFloat("hz", result[9])
    imu_node.setInt("status", result[10])

def unpack_imu_v2(buf):
    result = struct.unpack(imu_v2_fmt, buf)
    print result
    imu_node.setFloat("timestamp", result[0])
    imu_node.setFloat("p_rad_sec", result[1])
    imu_node.setFloat("q_rad_sec", result[2])
    imu_node.setFloat("r_rad_sec", result[3])
    imu_node.setFloat("ax_mps_sec", result[4])
    imu_node.setFloat("ay_mps_sec", result[5])
    imu_node.setFloat("az_mps_sec", result[6])
    imu_node.setFloat("hx", result[7])
    imu_node.setFloat("hy", result[8])
    imu_node.setFloat("hz", result[9])
    imu_node.setFloat("temp_C", result[10] / 10.0)
    imu_node.setInt("status", result[10])
    
def pack_airdata_v3():
    buf = struct.pack(airdata_v3_fmt,
                      airdata_node.getFloat("timestamp"),
                      int(airdata_node.getFloat("pressure_mbar") * 10.0),
                      int(airdata_node.getFloat("temp_degC") * 10.0),
                      int(vel_node.getFloat("airspeed_smoothed_kt") * 100.0),
                      pos_pressure_node.getFloat("altitude_smoothed_m"),
                      pos_combined_node.getFloat("altitude_true_m"),
                      int(vel_node.getFloat("pressure_vertical_speed_fps") * 60 * 10),
                      0, # empty
                      int(wind_node.getFloat("wind_dir_deg") * 100),
                      int(wind_node.getFloat("wind_speed_kt") * 4),
                      int(wind_node.getFloat("pitot_scale_factor") * 100),
                      airdata_node.getLong("status"))
    return (buf, airdata_v3_size)

def unpack_airdata_v1(buf):
    result = struct.unpack(airdata_v1_fmt, buf)
    print result
    airdata_node.setFloat("timestamp", result[0])
    vel_node.setFloat("airspeed_smoothed_kt", result[1] / 100.0)
    pos_pressure_node.setFloat("altitude_smoothed_m", result[2])
    vel_node.setFloat("pressure_vertical_speed_fps", (result[3] / 10.0) / 60.0)
    airdata_node.setFloat("acceleration", result[4] / 100.0)
    airdata_node.setInt("status", result[5])
    
def unpack_airdata_v2(buf):
    result = struct.unpack(airdata_v2_fmt, buf)
    print result
    airdata_node.setFloat("timestamp", result[0])
    vel_node.setFloat("airspeed_smoothed_kt", result[1] / 100.0)
    pos_pressure_node.setFloat("altitude_smoothed_m", result[2])
    vel_node.setFloat("pressure_vertical_speed_fps", (result[3] / 10.0) / 60.0)
    airdata_node.setFloat("acceleration", result[4] / 100.0)
    wind_node.setFloat("wind_dir_deg", result[5] / 100.0)
    wind_node.setFloat("wind_speed_kt", result[6] / 4.0)
    wind_node.setFloat("pitot_scale_factor", result[7] / 100.0)
    airdata_node.setInt("status", result[8])
    
def unpack_airdata_v3(buf):
    result = struct.unpack(airdata_v3_fmt, buf)
    print result
    airdata_node.setFloat("timestamp", result[0])
    airdata_node.setFloat("pressure_mbar", result[1] / 10.0)
    airdata_node.setFloat("temp_decC", result[2] / 10.0)
    vel_node.setFloat("airspeed_smoothed_kt", result[3] / 100.0)
    pos_pressure_node.setFloat("altitude_smoothed_m", result[4])
    vel_node.setFloat("pressure_vertical_speed_fps", (result[5] / 10.0) / 60.0)
    airdata_node.setFloat("acceleration", result[6] / 100.0)
    wind_node.setFloat("wind_dir_deg", result[7] / 100.0)
    wind_node.setFloat("wind_speed_kt", result[8] / 4.0)
    wind_node.setFloat("pitot_scale_factor", result[9] / 100.0)
    airdata_node.setInt("status", result[10])

def unpack_airdata_v4(buf):
    result = struct.unpack(airdata_v4_fmt, buf)
    print result
    airdata_node.setFloat("timestamp", result[0])
    airdata_node.setFloat("pressure_mbar", result[1] / 10.0)
    airdata_node.setFloat("temp_decC", result[2] / 10.0)
    vel_node.setFloat("airspeed_smoothed_kt", result[3] / 100.0)
    pos_pressure_node.setFloat("altitude_smoothed_m", result[4])
    pos_combined_node.setFloat("altitude_true_m", result[5])
    vel_node.setFloat("pressure_vertical_speed_fps", (result[6] / 10.0) / 60.0)
    airdata_node.setFloat("acceleration", result[7] / 100.0)
    wind_node.setFloat("wind_dir_deg", result[8] / 100.0)
    wind_node.setFloat("wind_speed_kt", result[9] / 4.0)
    wind_node.setFloat("pitot_scale_factor", result[10] / 100.0)
    airdata_node.setInt("status", result[11])

# FIXME: airdata v5: range on temp, drop acceleration/empty field

def pack_filter_v1():
    buf = struct.pack(filter_v1_fmt,
                      filter_node.getFloat("timestamp"),
                      filter_node.getFloat("latitude_deg"),
                      filter_node.getFloat("longitude_deg"),
                      filter_node.getFloat("altitude_m"),
                      int(filter_node.getFloat("vn_ms") * 100),
                      int(filter_node.getFloat("ve_ms") * 100),
                      int(filter_node.getFloat("vd_ms") * 100),
                      int(filter_node.getFloat("roll_deg") * 10),
                      int(filter_node.getFloat("pitch_deg") * 10),
                      int(filter_node.getFloat("heading_deg") * 10),
                      remote_link_node.getInt("sequence_num"),
                      0)
    return (buf, filter_v1_size)

def unpack_filter_v1(buf):
    result = struct.unpack(filter_v1_fmt, buf)
    print result
    filter_node.setFloat("timestamp", result[0])
    filter_node.setFloat("latitude_deg", result[1])
    filter_node.setFloat("longitude_deg", result[2])
    filter_node.setFloat("altitude_m", result[3])
    filter_node.setFloat("vn_ms", result[4] / 100.0)
    filter_node.setFloat("ve_ms", result[5] / 100.0)
    filter_node.setFloat("vd_ms", result[6] / 100.0)
    filter_node.setFloat("roll_deg", result[7] / 10.0)
    filter_node.setFloat("pitch_deg", result[8] / 10.0)
    filter_node.setFloat("heading_deg", result[9] / 10.0)
    remote_link_node.setInt("sequence_num", result[10])
    filter_node.setInt("status", result[11])
    

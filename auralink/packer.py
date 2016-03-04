import struct

from props import root, getNode

# FIXME: we are hard coding status flag to zero in many places which
# means we aren't using them properly (and/or wasting bytes)
ft2m = 0.3048
m2ft = 1.0 / ft2m

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
remote_link_node = getNode("/comms/remote_link", True)
filter_v1_fmt = "<dddfhhhhhhBB"
filter_v1_size = struct.calcsize(filter_v1_fmt)

NUM_ACTUATORS = 8
act_node = getNode("/actuators/actuator", True)
act_node.setLen("channel", NUM_ACTUATORS, 0.0)
act_v1_fmt = "<dhhHhhhhhB"
act_v1_size = struct.calcsize(act_v1_fmt)

pilot_node = getNode("/sensors/pilot_input", True)
pilot_node.setLen("channel", NUM_ACTUATORS, 0.0)
pilot_v1_fmt = "<dhhHhhhhhB"
pilot_v1_size = struct.calcsize(pilot_v1_fmt)

targets_node = getNode("/autopilot/targets", True)
route_node = getNode("/task/route", True)
ap_status_v1_fmt = "<dhhHhhhhddHHB"
ap_status_v1_size = struct.calcsize(ap_status_v1_fmt)
ap_status_v2_fmt = "<dhhHhhhhHddHHB"
ap_status_v2_size = struct.calcsize(ap_status_v2_fmt)

status_node = getNode("/status", True)
apm2_node = getNode("/sensors/APM2", True)
system_health_v1_fmt = "<dHH"
system_health_v1_size = struct.calcsize(system_health_v1_fmt)
system_health_v2_fmt = "<dHHHHH"
system_health_v2_size = struct.calcsize(system_health_v2_fmt)
system_health_v3_fmt = "<dHHHHHH"
system_health_v3_size = struct.calcsize(system_health_v3_fmt)

payload_node = getNode("/payload", True)
payload_v1_fmt = "<dH"
payload_v1_size = struct.calcsize(payload_v1_fmt)

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
    
def pack_act_v1():
    buf = struct.pack(act_v1_fmt,
                      act_node.getFloat("timestamp"),
                      int(act_node.getFloatEnum("channel", 0) * 30000),
                      int(act_node.getFloatEnum("channel", 1) * 30000),
                      int(act_node.getFloatEnum("channel", 2) * 60000),
                      int(act_node.getFloatEnum("channel", 3) * 30000),
                      int(act_node.getFloatEnum("channel", 4) * 30000),
                      int(act_node.getFloatEnum("channel", 5) * 30000),
                      int(act_node.getFloatEnum("channel", 6) * 30000),
                      int(act_node.getFloatEnum("channel", 7) * 30000),
                      0)
    return (buf, act_v1_size)

def unpack_act_v1(buf):
    result = struct.unpack(act_v1_fmt, buf)
    print result
    act_node.setFloat("timestamp", result[0])
    act_node.setFloatEnum("channel", 0, result[1] / 30000.0)
    act_node.setFloatEnum("channel", 1, result[2] / 30000.0)
    act_node.setFloatEnum("channel", 2, result[3] / 60000.0)
    act_node.setFloatEnum("channel", 3, result[4] / 30000.0)
    act_node.setFloatEnum("channel", 4, result[5] / 30000.0)
    act_node.setFloatEnum("channel", 5, result[6] / 30000.0)
    act_node.setFloatEnum("channel", 6, result[7] / 30000.0)
    act_node.setFloatEnum("channel", 7, result[8] / 30000.0)
    act_node.setInt("status", result[9])
    
def pack_pilot_v1():
    buf = struct.pack(pilot_v1_fmt,
                      pilot_node.getFloat("timestamp"),
                      int(pilot_node.getFloat("aileron") * 30000),
                      int(pilot_node.getFloat("elevator") * 30000),
                      int(pilot_node.getFloat("throttle") * 60000),
                      int(pilot_node.getFloat("rudder") * 30000),
                      int(pilot_node.getFloat("manual") * 30000),
                      int(pilot_node.getFloatEnum("channel", 5) * 30000),
                      int(pilot_node.getFloatEnum("channel", 6) * 30000),
                      int(pilot_node.getFloatEnum("channel", 7) * 30000),
                      0)
    return (buf, pilot_v1_size)

def unpack_pilot_v1(buf):
    result = struct.unpack(pilot_v1_fmt, buf)
    print result
    pilot_node.setFloat("timestamp", result[0])
    pilot_node.setFloat("aileron", result[1] / 30000.0)
    pilot_node.setFloat("elevator", result[2] / 30000.0)
    pilot_node.setFloat("throttle", result[3] / 60000.0)
    pilot_node.setFloat("rudder", result[4] / 30000.0)
    pilot_node.setFloat("manual", result[5] / 30000.0)
    pilot_node.setFloatEnum("channel", 5, result[6] / 30000.0)
    pilot_node.setFloatEnum("channel", 6, result[7] / 30000.0)
    pilot_node.setFloatEnum("channel", 7, result[8] / 30000.0)
    pilot_node.setInt("status", result[9])
    
def pack_ap_status_v2():
    target_agl_ft = targets_node.getFloat("altitude_agl_ft")
    ground_m = pos_pressure_node.getFloat("altitude_ground_m")
    error_m = pos_pressure_node.getFloat("pressure_error_m")
    target_msl_ft = (ground_m + error_m) * m2ft + target_agl_ft
    
    buf = struct.pack(ap_status_v1_fmt,
                      imu_node.getFloat("timestamp"),
                      int(targets_node.getFloat("groundtrack_deg") * 10),
                      int(targets_node.getFloat("roll_deg") * 10),
                      int(target_msl_ft),
                      int(targets_node.getFloat("climb_rate_fps") * 10),
                      int(targets_node.getFloat("pitch_deg") * 10),
                      int(targets_node.getFloat("the_dot") * 1000),
                      int(targets_node.getFloat("airspeed_kt") * 10),
                      route_node.getInt("target_waypoint_idx"), # FIXME
                      route_node.getFloat("target_lon"), # FIXME
                      route_node.getFloat("target_lat"), # FIXME
                      route_node.getInt("index"), # FIXME
                      route_node.getInt("size"), # FIXME
                      remote_link_node.getInt("sequence_num"))
    return (buf, ap_status_v1_size)

def unpack_ap_status_v1(buf):
    result = struct.unpack(ap_status_v1_fmt, buf)
    print result
    #ap_status_node.setFloat("timestamp", result[0]) # FIXME (where should this go?)
    targets_node.setFloat("groundtrack_deg", result[1] / 10.0)
    targets_node.setFloat("roll_deg", result[2] / 10.0)
    targets_node.setFloat("altitude_msl_ft", result[3])
    targets_node.setFloat("climb_rate_fps", result[4] / 10.0)
    targets_node.setFloat("pitch_deg", result[5] / 10.0)
    targets_node.setFloat("airspeed_kt", result[6] / 10.0)
    route_node.setInt("target_waypoint_idx", result[7]) # FIXME
    route_node.setFloat("target_lon", result[8]) # FIXME
    route_node.setFloat("target_lat", result[9]) # FIXME
    route_node.setInt("index", result[10]) # FIXME
    route_node.setInt("size", result[11]) # FIXME
    remote_link_node.setInt("sequence_num", result[12])
    
def unpack_ap_status_v2(buf):
    result = struct.unpack(ap_status_v2_fmt, buf)
    print result
    #ap_status_node.setFloat("timestamp", result[0]) #FIXME? where should this go
    targets_node.setFloat("groundtrack_deg", result[1] / 10.0) # FIXME?
    targets_node.setFloat("roll_deg", result[2] / 10.0)
    targets_node.setFloat("altitude_msl_ft", result[3])
    targets_node.setFloat("climb_rate_fps", result[4] / 10.0)
    targets_node.setFloat("pitch_deg", result[5] / 10.0)
    targets_node.setFloat("theta_dot", result[6] / 1000.0)
    targets_node.setFloat("airspeed_kt", result[7] / 10.0)
    route_node.setInt("target_waypoint_idx", result[8]) # FIXME
    route_node.setFloat("target_lon", result[9]) # FIXME
    route_node.setFloat("target_lat", result[10]) # FIXME
    route_node.setInt("index", result[11]) # FIXME
    route_node.setInt("size", result[12]) # FIXME
    remote_link_node.setInt("sequence_num", result[13])
    
def pack_system_health_v3():
    buf = struct.pack(system_health_v3_fmt,
                      imu_node.getFloat("timestamp"),
                      int(status_node.getFloat("system_load_avg") * 100),
                      int(apm2_node.getFloat("board_vcc") * 1000),
                      int(apm2_node.getFloat("extern_volt") * 1000),
                      int(apm2_node.getFloat("extern_cell_volt") * 1000),
                      int(apm2_node.getFloat("extern_amps") * 1000),
                      int(apm2_node.getFloat("extern_current_mah")))
    return (buf, system_health_v3_size)

def unpack_system_health_v1(buf):
    result = struct.unpack(system_health_v1_fmt, buf)
    print result
    # imu_node.setFloat("timestamp", result[0]) # fixme? where to write this value?
    apm2_node.setFloat("board_vcc", result[1] / 1000.0)
    status_node.setFloat("system_load_avg", result[2] / 100.0)

def unpack_system_health_v2(buf):
    result = struct.unpack(system_health_v2_fmt, buf)
    print result
    # imu_node.setFloat("timestamp", result[0]) # fixme? where to write this value?
    status_node.setFloat("system_load_avg", result[1] / 100.0)
    apm2_node.setFloat("board_vcc", result[2] / 1000.0)
    apm2_node.setFloat("extern_volt", result[3] / 1000.0)
    apm2_node.setFloat("extern_amps", result[5] / 1000.0)
    apm2_node.setFloat("extern_current_mah", result[6])

def unpack_system_health_v3(buf):
    result = struct.unpack(system_health_v3_fmt, buf)
    print result
    # imu_node.setFloat("timestamp", result[0]) # fixme? where to write this value?
    status_node.setFloat("system_load_avg", result[1] / 100.0)
    apm2_node.setFloat("board_vcc", result[2] / 1000.0)
    apm2_node.setFloat("extern_volt", result[3] / 1000.0)
    apm2_node.setFloat("extern_cell_volt", result[4] / 1000.0)
    apm2_node.setFloat("extern_amps", result[5] / 1000.0)
    apm2_node.setFloat("extern_current_mah", result[6])

def pack_payload_v1():
    buf = struct.pack(payload_v1_fmt,
                      imu_node.getFloat("timestamp"),
                      payload_node.getFloat("trigger_num"))
    return (buf, payload_v1_size)

def unpack_payload_v1(buf):
    result = struct.unpack(payload_v1_fmt, buf)
    print result
    # payload_node.setFloat("timestamp", result[0]) # FIXME: where to write?
    payload_node.setInt("trigger_num", result[1])
    
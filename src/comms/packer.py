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

imu_timestamp = 0.0

gps_nodes = []
gps_v1_fmt = '<dddfhhhdBB'
gps_v1_size = struct.calcsize(gps_v1_fmt)
gps_v2_fmt = '<BdddfhhhdBB'
gps_v2_size = struct.calcsize(gps_v2_fmt)

imu_nodes = []
imu_v1_fmt = '<dfffffffffB'
imu_v1_size = struct.calcsize(imu_v1_fmt)
imu_v2_fmt = '<dfffffffffhB'
imu_v2_size = struct.calcsize(imu_v2_fmt)
imu_v3_fmt = '<BdfffffffffhB'
imu_v3_size = struct.calcsize(imu_v3_fmt)

airdata_nodes = []
pos_node = getNode("/position", True)
pos_pressure_node = getNode("/position/pressure", True)
pos_combined_node = getNode("/position/combined", True)
vel_node = getNode("/velocity", True)
wind_node = getNode("/filters/wind", True)
airdata_v3_fmt = "<dHhhfhhHBBB"
airdata_v3_size = struct.calcsize(airdata_v3_fmt)
airdata_v4_fmt = "<dHhhffhhHBBB"
airdata_v4_size = struct.calcsize(airdata_v4_fmt)
airdata_v5_fmt = "<BdHhhffhHBBB"
airdata_v5_size = struct.calcsize(airdata_v5_fmt)

filter_node = getNode("/filters/filter", True)
remote_link_node = getNode("/comms/remote_link", True)
filter_v1_fmt = "<dddfhhhhhhBB"
filter_v1_size = struct.calcsize(filter_v1_fmt)

NUM_ACTUATORS = 8
act_nodes = []
act_v1_fmt = "<dhhHhhhhhB"
act_v1_size = struct.calcsize(act_v1_fmt)
act_v2_fmt = "<BdhhHhhhhhB"
act_v2_size = struct.calcsize(act_v2_fmt)

pilot_nodes = []
pilot_v1_fmt = "<dhhHhhhhhB"
pilot_v1_size = struct.calcsize(pilot_v1_fmt)
pilot_v2_fmt = "<BdhhHhhhhhB"
pilot_v2_size = struct.calcsize(pilot_v2_fmt)

targets_node = getNode("/autopilot/targets", True)
route_node = getNode("/task/route", True)
ap_status_v1_fmt = "<dhhHhhhhddHHB"
ap_status_v1_size = struct.calcsize(ap_status_v1_fmt)
ap_status_v2_fmt = "<dhhHhhhhHddHHB"
ap_status_v2_size = struct.calcsize(ap_status_v2_fmt)

status_node = getNode("/status", True)
apm2_node = getNode("/sensors/APM2", True)
system_health_v2_fmt = "<dHHHHH"
system_health_v2_size = struct.calcsize(system_health_v2_fmt)
system_health_v3_fmt = "<dHHHHHH"
system_health_v3_size = struct.calcsize(system_health_v3_fmt)
system_health_v4_fmt = "<BdHHHHHH"
system_health_v4_size = struct.calcsize(system_health_v4_fmt)

payload_node = getNode("/payload", True)
payload_v1_fmt = "<dH"
payload_v1_size = struct.calcsize(payload_v1_fmt)

def init():
    pass

def pack_gps_v2(index):
    if index >= len(gps_nodes):
        for i in range(len(gps_nodes),index+1):
            path = '/sensors/gps[%d]' % i
            gps_nodes.append( getNode(path, True) )
    node = gps_nodes[index]
    buf = struct.pack(gps_v2_fmt,
                      index,
                      node.getFloat("timestamp"),
                      node.getFloat("latitude_deg"),
                      node.getFloat("longitude_deg"),
                      node.getFloat("altitude_m"),
                      int(node.getFloat("vn_ms") * 100),
                      int(node.getFloat("ve_ms") * 100),
                      int(node.getFloat("vd_ms") * 100),
                      node.getFloat("unix_time_sec"),
                      node.getInt("satellites"),
                      0)
    return buf

def unpack_gps_v1(buf):
    index = 0
    if index >= len(gps_nodes):
        for i in range(len(gps_nodes),index+1):
            path = '/sensors/gps[%d]' % i
            gps_nodes.append( getNode(path, True) )
    node = gps_nodes[index]
    
    result = struct.unpack(gps_v1_fmt, buf)
    print result
    
    node.setFloat("timestamp", result[0])
    node.setFloat("latitude_deg", result[1])
    node.setFloat("longitude_deg", result[2])
    node.setFloat("altitude_m", result[3])
    node.setFloat("vn_ms", result[4] / 100.0)
    node.setFloat("ve_ms", result[5] / 100.0)
    node.setFloat("vd_ms", result[6] / 100.0)
    node.setFloat("unix_time_sec", result[7])
    node.setInt("satellites", result[8])
    node.setInt("status", result[9])
    
def unpack_gps_v2(buf):
    result = struct.unpack(gps_v2_fmt, buf)
    print result

    index = result[0]
    if index >= len(gps_nodes):
        for i in range(len(gps_nodes),index+1):
            path = '/sensors/gps[%d]' % i
            gps_nodes.append( getNode(path, True) )
    node = gps_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setFloat("latitude_deg", result[2])
    node.setFloat("longitude_deg", result[3])
    node.setFloat("altitude_m", result[4])
    node.setFloat("vn_ms", result[5] / 100.0)
    node.setFloat("ve_ms", result[6] / 100.0)
    node.setFloat("vd_ms", result[7] / 100.0)
    node.setFloat("unix_time_sec", result[8])
    node.setInt("satellites", result[9])
    node.setInt("status", result[10])
    
def pack_imu_v3(index):
    if index >= len(imu_nodes):
        for i in range(len(imu_nodes),index+1):
            path = '/sensors/imu[%d]' % i
            imu_nodes.append( getNode(path, True) )
    node = imu_nodes[index]

    imu_timestamp = node.getFloat("timestamp")
    
    buf = struct.pack(imu_v3_fmt,
                      index,
                      imu_timestamp,
                      node.getFloat("p_rad_sec"),
                      node.getFloat("q_rad_sec"),
                      node.getFloat("r_rad_sec"),
                      node.getFloat("ax_mps_sec"),
                      node.getFloat("ay_mps_sec"),
                      node.getFloat("az_mps_sec"),
                      node.getFloat("hx"),
                      node.getFloat("hy"),
                      node.getFloat("hz"),
                      int(node.getFloat("temp_C") * 10.0),
                      0)
    return buf

def unpack_imu_v1(buf):
    index = 0
    if index >= len(imu_nodes):
        for i in range(len(imu_nodes),index+1):
            path = '/sensors/imu[%d]' % i
            imu_nodes.append( getNode(path, True) )
    node = imu_nodes[index]

    result = struct.unpack(imu_v1_fmt, buf)
    print result
    
    node.setFloat("timestamp", result[0])
    node.setFloat("p_rad_sec", result[1])
    node.setFloat("q_rad_sec", result[2])
    node.setFloat("r_rad_sec", result[3])
    node.setFloat("ax_mps_sec", result[4])
    node.setFloat("ay_mps_sec", result[5])
    node.setFloat("az_mps_sec", result[6])
    node.setFloat("hx", result[7])
    node.setFloat("hy", result[8])
    node.setFloat("hz", result[9])
    node.setInt("status", result[10])

def unpack_imu_v2(buf):
    index = 0
    if index >= len(imu_nodes):
        for i in range(len(imu_nodes),index+1):
            path = '/sensors/imu[%d]' % i
            imu_nodes.append( getNode(path, True) )
    node = imu_nodes[index]

    result = struct.unpack(imu_v2_fmt, buf)
    print result

    node.setFloat("timestamp", result[0])
    node.setFloat("p_rad_sec", result[1])
    node.setFloat("q_rad_sec", result[2])
    node.setFloat("r_rad_sec", result[3])
    node.setFloat("ax_mps_sec", result[4])
    node.setFloat("ay_mps_sec", result[5])
    node.setFloat("az_mps_sec", result[6])
    node.setFloat("hx", result[7])
    node.setFloat("hy", result[8])
    node.setFloat("hz", result[9])
    node.setFloat("temp_C", result[10] / 10.0)
    node.setInt("status", result[10])
    
def unpack_imu_v3(buf):
    result = struct.unpack(imu_v3_fmt, buf)
    print result

    index = result[0]
    if index >= len(imu_nodes):
        for i in range(len(imu_nodes),index+1):
            path = '/sensors/imu[%d]' % i
            imu_nodes.append( getNode(path, True) )
    node = imu_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setFloat("p_rad_sec", result[2])
    node.setFloat("q_rad_sec", result[3])
    node.setFloat("r_rad_sec", result[4])
    node.setFloat("ax_mps_sec", result[5])
    node.setFloat("ay_mps_sec", result[6])
    node.setFloat("az_mps_sec", result[7])
    node.setFloat("hx", result[8])
    node.setFloat("hy", result[9])
    node.setFloat("hz", result[10])
    node.setFloat("temp_C", result[11] / 10.0)
    node.setInt("status", result[12])
    
def pack_airdata_v5(index):
    if index >= len(airdata_nodes):
        for i in range(len(airdata_nodes),index+1):
            path = '/sensors/airdata[%d]' % i
            airdata_nodes.append( getNode(path, True) )
    node = airdata_nodes[index]
    
    buf = struct.pack(airdata_v5_fmt,
                      index,
                      node.getFloat("timestamp"),
                      int(node.getFloat("pressure_mbar") * 10.0),
                      int(node.getFloat("temp_degC") * 100.0),
                      int(vel_node.getFloat("airspeed_smoothed_kt") * 100.0),
                      pos_pressure_node.getFloat("altitude_smoothed_m"),
                      pos_combined_node.getFloat("altitude_true_m"),
                      int(vel_node.getFloat("pressure_vertical_speed_fps") * 60 * 10),
                      int(wind_node.getFloat("wind_dir_deg") * 100),
                      int(wind_node.getFloat("wind_speed_kt") * 4),
                      int(wind_node.getFloat("pitot_scale_factor") * 100),
                      node.getInt("status"))
    return buf

def unpack_airdata_v3(buf):
    index = 0
    if index >= len(airdata_nodes):
        for i in range(len(airdata_nodes),index+1):
            path = '/sensors/airdata[%d]' % i
            airdata_nodes.append( getNode(path, True) )
    node = airdata_nodes[index]

    result = struct.unpack(airdata_v3_fmt, buf)
    print result
    
    node.setFloat("timestamp", result[0])
    node.setFloat("pressure_mbar", result[1] / 10.0)
    node.setFloat("temp_degC", result[2] / 10.0)
    vel_node.setFloat("airspeed_smoothed_kt", result[3] / 100.0)
    pos_pressure_node.setFloat("altitude_smoothed_m", result[4])
    vel_node.setFloat("pressure_vertical_speed_fps", (result[5] / 10.0) / 60.0)
    node.setFloat("acceleration", result[6] / 100.0)
    wind_node.setFloat("wind_dir_deg", result[7] / 100.0)
    wind_node.setFloat("wind_speed_kt", result[8] / 4.0)
    wind_node.setFloat("pitot_scale_factor", result[9] / 100.0)
    node.setInt("status", result[10])

def unpack_airdata_v4(buf):
    index = 0
    if index >= len(airdata_nodes):
        for i in range(len(airdata_nodes),index+1):
            path = '/sensors/airdata[%d]' % i
            airdata_nodes.append( getNode(path, True) )
    node = airdata_nodes[index]

    result = struct.unpack(airdata_v4_fmt, buf)
    print result
    
    node.setFloat("timestamp", result[0])
    node.setFloat("pressure_mbar", result[1] / 10.0)
    node.setFloat("temp_degC", result[2] / 10.0)
    vel_node.setFloat("airspeed_smoothed_kt", result[3] / 100.0)
    pos_pressure_node.setFloat("altitude_smoothed_m", result[4])
    pos_combined_node.setFloat("altitude_true_m", result[5])
    vel_node.setFloat("pressure_vertical_speed_fps", (result[6] / 10.0) / 60.0)
    node.setFloat("acceleration", result[7] / 100.0)
    wind_node.setFloat("wind_dir_deg", result[8] / 100.0)
    wind_node.setFloat("wind_speed_kt", result[9] / 4.0)
    wind_node.setFloat("pitot_scale_factor", result[10] / 100.0)
    node.setInt("status", result[11])

def unpack_airdata_v5(buf):
    result = struct.unpack(airdata_v5_fmt, buf)
    print result
    
    index = result[0]
    if index >= len(airdata_nodes):
        for i in range(len(airdata_nodes),index+1):
            path = '/sensors/airdata[%d]' % i
            airdata_nodes.append( getNode(path, True) )
    node = airdata_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setFloat("pressure_mbar", result[2] / 10.0)
    node.setFloat("temp_degC", result[3] / 100.0)
    vel_node.setFloat("airspeed_smoothed_kt", result[4] / 100.0)
    pos_pressure_node.setFloat("altitude_smoothed_m", result[5])
    pos_combined_node.setFloat("altitude_true_m", result[6])
    vel_node.setFloat("pressure_vertical_speed_fps", (result[7] / 10.0) / 60.0)
    wind_node.setFloat("wind_dir_deg", result[8] / 100.0)
    wind_node.setFloat("wind_speed_kt", result[9] / 4.0)
    wind_node.setFloat("pitot_scale_factor", result[10] / 100.0)
    node.setInt("status", result[11])

def pack_filter_v1(index):
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
    return buf

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
    
def pack_act_v2(index):
    if index >= len(act_nodes):
        for i in range(len(act_nodes),index+1):
            path = '/actuators/actuator[%d]' % i
            node = getNode(path, True)
            node.setLen("channel", NUM_ACTUATORS, 0.0)
            act_nodes.append( node )
    node = act_nodes[index]
    
    buf = struct.pack(act_v2_fmt,
                      index,
                      node.getFloat("timestamp"),
                      int(node.getFloatEnum("channel", 0) * 30000),
                      int(node.getFloatEnum("channel", 1) * 30000),
                      int(node.getFloatEnum("channel", 2) * 60000),
                      int(node.getFloatEnum("channel", 3) * 30000),
                      int(node.getFloatEnum("channel", 4) * 30000),
                      int(node.getFloatEnum("channel", 5) * 30000),
                      int(node.getFloatEnum("channel", 6) * 30000),
                      int(node.getFloatEnum("channel", 7) * 30000),
                      0)
    return buf

def unpack_act_v1(buf):
    index = 0
    if index >= len(act_nodes):
        for i in range(len(act_nodes),index+1):
            path = '/actuators/actuator[%d]' % i
            node = getNode(path, True)
            node.setLen("channel", NUM_ACTUATORS, 0.0)
            act_nodes.append( node )
    node = act_nodes[index]
    
    result = struct.unpack(act_v1_fmt, buf)
    print result

    node.setFloat("timestamp", result[0])
    node.setFloatEnum("channel", 0, result[1] / 30000.0)
    node.setFloatEnum("channel", 1, result[2] / 30000.0)
    node.setFloatEnum("channel", 2, result[3] / 60000.0)
    node.setFloatEnum("channel", 3, result[4] / 30000.0)
    node.setFloatEnum("channel", 4, result[5] / 30000.0)
    node.setFloatEnum("channel", 5, result[6] / 30000.0)
    node.setFloatEnum("channel", 6, result[7] / 30000.0)
    node.setFloatEnum("channel", 7, result[8] / 30000.0)
    node.setInt("status", result[9])
    
def unpack_act_v2(buf):
    result = struct.unpack(act_v2_fmt, buf)
    print result
    
    index = result[0]
    if index >= len(act_nodes):
        for i in range(len(act_nodes),index+1):
            path = '/actuators/actuator[%d]' % i
            node = getNode(path, True)
            node.setLen("channel", NUM_ACTUATORS, 0.0)
            act_nodes.append( node )
    node = act_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setFloatEnum("channel", 0, result[2] / 30000.0)
    node.setFloatEnum("channel", 1, result[3] / 30000.0)
    node.setFloatEnum("channel", 2, result[4] / 60000.0)
    node.setFloatEnum("channel", 3, result[5] / 30000.0)
    node.setFloatEnum("channel", 4, result[6] / 30000.0)
    node.setFloatEnum("channel", 5, result[7] / 30000.0)
    node.setFloatEnum("channel", 6, result[8] / 30000.0)
    node.setFloatEnum("channel", 7, result[9] / 30000.0)
    node.setInt("status", result[10])
    
def pack_pilot_v2(index):
    if index >= len(pilot_nodes):
        for i in range(len(pilot_nodes),index+1):
            path = '/sensors/pilot_input[%d]' % i
            node = getNode(path, True)
            node.setLen("channel", NUM_ACTUATORS, 0.0)
            pilot_nodes.append( node )
    node = pilot_nodes[index]
    
    buf = struct.pack(pilot_v2_fmt,
                      index,
                      node.getFloat("timestamp"),
                      int(node.getFloat("aileron") * 30000),
                      int(node.getFloat("elevator") * 30000),
                      int(node.getFloat("throttle") * 60000),
                      int(node.getFloat("rudder") * 30000),
                      int(node.getFloat("manual") * 30000),
                      int(node.getFloatEnum("channel", 5) * 30000),
                      int(node.getFloatEnum("channel", 6) * 30000),
                      int(node.getFloatEnum("channel", 7) * 30000),
                      0)
    return buf

def unpack_pilot_v1(buf):
    index = 0
    if index >= len(pilot_nodes):
        for i in range(len(pilot_nodes),index+1):
            path = '/sensors/pilot_input[%d]' % i
            node = getNode(path, True)
            node.setLen("channel", NUM_ACTUATORS, 0.0)
            pilot_nodes.append( node )
    node = pilot_nodes[index]

    result = struct.unpack(pilot_v1_fmt, buf)
    print result
    node.setFloat("timestamp", result[0])
    node.setFloat("aileron", result[1] / 30000.0)
    node.setFloat("elevator", result[2] / 30000.0)
    node.setFloat("throttle", result[3] / 60000.0)
    node.setFloat("rudder", result[4] / 30000.0)
    node.setFloat("manual", result[5] / 30000.0)
    node.setFloatEnum("channel", 5, result[6] / 30000.0)
    node.setFloatEnum("channel", 6, result[7] / 30000.0)
    node.setFloatEnum("channel", 7, result[8] / 30000.0)
    node.setInt("status", result[9])
    
def unpack_pilot_v2(buf):
    result = struct.unpack(pilot_v2_fmt, buf)
    print result
    
    index = result[0]
    if index >= len(pilot_nodes):
        for i in range(len(pilot_nodes),index+1):
            path = '/sensors/pilot_input[%d]' % i
            node = getNode(path, True)
            node.setLen("channel", NUM_ACTUATORS, 0.0)
            pilot_nodes.append( node )
    node = pilot_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setFloat("aileron", result[2] / 30000.0)
    node.setFloat("elevator", result[3] / 30000.0)
    node.setFloat("throttle", result[4] / 60000.0)
    node.setFloat("rudder", result[5] / 30000.0)
    node.setFloat("manual", result[6] / 30000.0)
    node.setFloatEnum("channel", 5, result[7] / 30000.0)
    node.setFloatEnum("channel", 6, result[8] / 30000.0)
    node.setFloatEnum("channel", 7, result[9] / 30000.0)
    node.setInt("status", result[10])
    
def pack_ap_status_v2(index):
    target_agl_ft = targets_node.getFloat("altitude_agl_ft")
    ground_m = pos_pressure_node.getFloat("altitude_ground_m")
    error_m = pos_pressure_node.getFloat("pressure_error_m")
    target_msl_ft = (ground_m + error_m) * m2ft + target_agl_ft
    
    buf = struct.pack(ap_status_v1_fmt,
                      imu_timestamp,
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
    return buf

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
    
def pack_system_health_v4(index):
    buf = struct.pack(system_health_v4_fmt,
                      index,
                      imu_timestamp,
                      int(status_node.getFloat("system_load_avg") * 100),
                      int(apm2_node.getFloat("board_vcc") * 1000),
                      int(apm2_node.getFloat("extern_volt") * 1000),
                      int(apm2_node.getFloat("extern_cell_volt") * 1000),
                      int(apm2_node.getFloat("extern_amps") * 1000),
                      int(apm2_node.getFloat("extern_current_mah")))
    return buf

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

def unpack_system_health_v4(buf):
    result = struct.unpack(system_health_v4_fmt, buf)
    print result

    index = result[0]
    
    # imu_node.setFloat("timestamp", result[1]) # fixme? where to write this value?
    status_node.setFloat("system_load_avg", result[2] / 100.0)
    apm2_node.setFloat("board_vcc", result[3] / 1000.0)
    apm2_node.setFloat("extern_volt", result[4] / 1000.0)
    apm2_node.setFloat("extern_cell_volt", result[5] / 1000.0)
    apm2_node.setFloat("extern_amps", result[6] / 1000.0)
    apm2_node.setFloat("extern_current_mah", result[7])

def pack_payload_v1(index):
    buf = struct.pack(payload_v1_fmt,
                      imu_timestamp,
                      payload_node.getFloat("trigger_num"))
    return buf

def unpack_payload_v1(buf):
    result = struct.unpack(payload_v1_fmt, buf)
    print result
    # payload_node.setFloat("timestamp", result[0]) # FIXME: where to write?
    payload_node.setInt("trigger_num", result[1])
    

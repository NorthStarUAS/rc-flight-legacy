import re
import struct

from props import getNode

from comms.packet_id import *

# FIXME: we are hard coding status flag to zero in many places which
# means we aren't using them properly (and/or wasting bytes)

ft2m = 0.3048
m2ft = 1.0 / ft2m

START_OF_MSG0 = 147
START_OF_MSG1 = 224

# python struct package notes:
#
# < means use little endian order, no byte alignment
# b = int8_t, B = uint8_t
# h = int16_t, H = uint16_t
# l = int32_t, L = uint32_t
# f = float, d = double
# p = pascal string (variable length with system adding len as first byte)

imu_timestamp = 0.0

gps_nodes = []
gps_v2_fmt = '<BdddfhhhdBB'
gps_v3_fmt = '<BdddfhhhdBHHHB'
gps_v4_fmt = '<BfddfhhhdBHHHB'

imu_nodes = []
imu_v3_fmt = '<BdfffffffffhB'
imu_v4_fmt = '<BffffffffffhB'

airdata_nodes = []
pos_node = getNode("/position", True)
pos_pressure_node = getNode("/position/pressure", True)
pos_combined_node = getNode("/position/combined", True)
vel_node = getNode("/velocity", True)
wind_node = getNode("/filters/wind", True)
airdata_v5_fmt = "<BdHhhffhHBBB"
airdata_v6_fmt = "<BfHhhffhHBBB"
airdata_v7_fmt = "<BfHhhffhHBBHB"

filter_nodes = []
remote_link_node = getNode("/comms/remote_link", True)
filter_v2_fmt = "<BdddfhhhhhhBB"
filter_v3_fmt = "<BdddfhhhhhhhhhhhhBB"
filter_v4_fmt = "<BfddfhhhhhhhhhhhhBB"

NUM_ACTUATORS = 8
act_node = getNode("/actuators", True)
act_v2_fmt = "<BdhhHhhhhhB"
act_v3_fmt = "<BfhhHhhhhhB"

pilot_nodes = []
pilot_v2_fmt = "<BdhhhhhhhhB"
pilot_v3_fmt = "<BfhhhhhhhhB"

status_node = getNode("/status", True)
ap_node = getNode("/autopilot", True)
targets_node = getNode("/autopilot/targets", True)
tecs_node = getNode("/autopilot/tecs", True)
task_node = getNode("/task", True)
route_node = getNode("/task/route", True)
active_node = getNode("/task/route/active", True)
home_node = getNode("/task/home", True)
circle_node = getNode("/task/circle", True)
ap_status_v4_fmt = "<BdhhHHhhHHddHHB"
ap_status_v5_fmt = "<BdBhhHhhhHHddHHB"
ap_status_v6_fmt = "<BdBhhHhhhHHddHHBHB"
ap_status_v7_fmt = "<BfBhhHhhhHHddHHBHB"

power_node = getNode("/sensors/power", True)
system_health_v4_fmt = "<BdHHHHHH"
system_health_v5_fmt = "<BfHHHHHH"

payload_node = getNode("/payload", True)
payload_v2_fmt = "<BdH"
payload_v3_fmt = "<BfH"

event_node = getNode("/status/event", True)
# event_v1_fmt = "<BdB%ds"
# this packet will be variable length so size and fmt string are dynamic

def init():
    pass

# simple 2-byte checksum
def compute_cksum(id, buf, size):
    c0 = 0
    c1 = 0
    c0 = (c0 + id) & 0xff
    c1 = (c1 + c0) & 0xff
    #print("c0 =", c0, "c1 =", c1)
    c0 = (c0 + size) & 0xff
    c1 = (c1 + c0) & 0xff
    #print("c0 =", c0, "c1 =", c1)
    for i in range(0, size):
        c0 = (c0 + buf[i]) & 0xff
        c1 = (c1 + c0) & 0xff
        #print("c0 =", c0, "c1 =", c1, i, '[', buf[i], ']')
    #print("c0 =", c0, "c1 =", c1)
    return (c0, c1)

# wrap payload in header bytes, id, length, payload, and compute checksums
def wrap_packet( packet_id, payload ):
    size = len(payload)
    buf = bytearray()
    buf.append(START_OF_MSG0)   # start of message sync bytes
    buf.append(START_OF_MSG1)   # start of message sync bytes
    buf.append(packet_id)       # packet id (1 byte)
    buf.append(size)            # packet size (1 byte)
    buf.extend(payload)         # copy payload
    (cksum0, cksum1) = compute_cksum( packet_id, payload, size)
    buf.append(cksum0)          # check sum byte 1
    buf.append(cksum1)          # check sum byte 2
    return buf
    
def pack_gps_bin(index):
    if index >= len(gps_nodes):
        for i in range(len(gps_nodes),index+1):
            path = '/sensors/gps[%d]' % i
            gps_nodes.append( getNode(path, True) )
    node = gps_nodes[index]
    buf = struct.pack(gps_v4_fmt,
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
                      int(node.getFloat('horiz_accuracy_m') * 100),
                      int(node.getFloat('vert_accuracy_m') * 100),
                      int(node.getFloat('pdop') * 100),
                      node.getInt('fixType'))
    return wrap_packet(GPS_PACKET_V4, buf)

def pack_gps_dict(index):
    gps_node = getNode('/sensors/gps[%d]' % index, True)
    row = dict()
    row['timestamp'] = gps_node.getFloat('timestamp')
    row['latitude_deg'] = gps_node.getFloat('latitude_deg')
    row['longitude_deg'] = gps_node.getFloat('longitude_deg')
    row['altitude_m'] = gps_node.getFloat('altitude_m')
    row['vn_ms'] = gps_node.getFloat('vn_ms')
    row['ve_ms'] = gps_node.getFloat('ve_ms')
    row['vd_ms'] = gps_node.getFloat('vd_ms')
    row['unix_time_sec'] = gps_node.getFloat('unix_time_sec')
    row['satellites'] = gps_node.getInt('satellites')
    row['horiz_accuracy_m'] = gps_node.getFloat('horiz_accuracy_m')
    row['vert_accuracy_m'] = gps_node.getFloat('vert_accuracy_m')
    row['pdop'] = gps_node.getFloat('pdop')
    row['fix_type'] = gps_node.getInt('fixType')
    return row
    
def pack_gps_csv(index):
    gps_node = getNode('/sensors/gps[%d]' % index, True)
    row = dict()
    row['timestamp'] = '%.4f' % gps_node.getFloat('timestamp')
    row['latitude_deg'] = '%.10f' % gps_node.getFloat('latitude_deg')
    row['longitude_deg'] = '%.10f' % gps_node.getFloat('longitude_deg')
    row['altitude_m'] = '%.2f' % gps_node.getFloat('altitude_m')
    row['vn_ms'] = '%.4f' % gps_node.getFloat('vn_ms')
    row['ve_ms'] = '%.4f' % gps_node.getFloat('ve_ms')
    row['vd_ms'] = '%.4f' % gps_node.getFloat('vd_ms')
    row['unix_time_sec'] = '%.3f' % gps_node.getFloat('unix_time_sec')
    row['satellites'] = '%d' % gps_node.getInt('satellites')
    row['horiz_accuracy_m'] = '%.2f' % gps_node.getFloat('horiz_accuracy_m')
    row['vert_accuracy_m'] = '%.2f' % gps_node.getFloat('vert_accuracy_m')
    row['pdop'] = '%.2f' % gps_node.getFloat('pdop')
    row['fix_type'] = '%d' % gps_node.getInt('fixType')
    keys =['timestamp', 'latitude_deg', 'longitude_deg', 'altitude_m',
           'vn_ms', 've_ms', 'vd_ms', 'unix_time_sec', 'satellites',
           'horiz_accuracy_m', 'vert_accuracy_m', 'pdop', 'fix_type']
    return row, keys
    
def unpack_gps_v2(buf):
    result = struct.unpack(gps_v2_fmt, buf)

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

    return index

def unpack_gps_v3(buf):
    result = struct.unpack(gps_v3_fmt, buf)

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
    node.setFloat('horiz_accuracy_m', result[10] / 100.0)
    node.setFloat('vert_accuracy_m', result[11] / 100.0)
    node.setFloat('pdop', result[12] / 100.0)
    node.setInt('fixType', result[13])
    node.setInt("status", 0)

    return index

def unpack_gps_v4(buf):
    result = struct.unpack(gps_v4_fmt, buf)

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
    node.setFloat('horiz_accuracy_m', result[10] / 100.0)
    node.setFloat('vert_accuracy_m', result[11] / 100.0)
    node.setFloat('pdop', result[12] / 100.0)
    node.setInt('fixType', result[13])
    node.setInt("status", 0)

    return index

def pack_imu_bin(index):
    global imu_timestamp
    
    if index >= len(imu_nodes):
        for i in range(len(imu_nodes),index+1):
            path = '/sensors/imu[%d]' % i
            imu_nodes.append( getNode(path, True) )
    node = imu_nodes[index]

    imu_timestamp = node.getFloat("timestamp")
    
    buf = struct.pack(imu_v4_fmt,
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
                      int(round(node.getFloat("temp_C") * 10.0)),
                      0)
    return wrap_packet(IMU_PACKET_V4, buf)

def pack_imu_dict(index):
    imu_node = getNode('/sensors/imu[%d]' % index, True)
    row = dict()
    row['timestamp'] = imu_node.getFloat('timestamp')
    row['p_rad_sec'] = imu_node.getFloat('p_rad_sec')
    row['q_rad_sec'] = imu_node.getFloat('q_rad_sec')
    row['r_rad_sec'] = imu_node.getFloat('r_rad_sec')
    row['ax_mps_sec'] = imu_node.getFloat('ax_mps_sec')
    row['ay_mps_sec'] = imu_node.getFloat('ay_mps_sec')
    row['az_mps_sec'] = imu_node.getFloat('az_mps_sec')
    row['hx'] = imu_node.getFloat('hx')
    row['hy'] = imu_node.getFloat('hy')
    row['hz'] = imu_node.getFloat('hz')
    row['temp_C'] = imu_node.getFloat('temp_C')
    row['status'] = imu_node.getInt('status')
    return row
    
def pack_imu_csv(index):
    imu_node = getNode('/sensors/imu[%d]' % index, True)
    row = dict()
    row['timestamp'] = '%.4f' % imu_node.getFloat('timestamp')
    row['p_rad_sec'] = '%.4f' % imu_node.getFloat('p_rad_sec')
    row['q_rad_sec'] = '%.4f' % imu_node.getFloat('q_rad_sec')
    row['r_rad_sec'] = '%.4f' % imu_node.getFloat('r_rad_sec')
    row['ax_mps_sec'] = '%.4f' % imu_node.getFloat('ax_mps_sec')
    row['ay_mps_sec'] = '%.4f' % imu_node.getFloat('ay_mps_sec')
    row['az_mps_sec'] = '%.4f' % imu_node.getFloat('az_mps_sec')
    row['hx'] = '%.3f' % imu_node.getFloat('hx')
    row['hy'] = '%.3f' % imu_node.getFloat('hy')
    row['hz'] = '%.3f' % imu_node.getFloat('hz')
    row['temp_C'] = '%.1f' % imu_node.getFloat('temp_C')
    row['status'] = '%d' % imu_node.getInt('status')
    keys = ['timestamp', 'p_rad_sec', 'q_rad_sec', 'r_rad_sec',
            'ax_mps_sec', 'ay_mps_sec', 'az_mps_sec',
            'hx', 'hy', 'hz', 'temp_C', 'status']
    return row, keys
    
def unpack_imu_v3(buf):
    result = struct.unpack(imu_v3_fmt, buf)

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

    return index

def unpack_imu_v4(buf):
    result = struct.unpack(imu_v4_fmt, buf)

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

    return index

def pack_airdata_bin(index):
    if index >= len(airdata_nodes):
        for i in range(len(airdata_nodes),index+1):
            path = '/sensors/airdata[%d]' % i
            airdata_nodes.append( getNode(path, True) )
    node = airdata_nodes[index]
    
    buf = struct.pack(airdata_v7_fmt,
                      index,
                      node.getFloat("timestamp"),
                      int(node.getFloat("pressure_mbar") * 10.0),
                      int(node.getFloat("temp_C") * 100.0),
                      int(vel_node.getFloat("airspeed_smoothed_kt") * 100.0),
                      pos_pressure_node.getFloat("altitude_smoothed_m"),
                      pos_combined_node.getFloat("altitude_true_m"),
                      int(vel_node.getFloat("pressure_vertical_speed_fps") * 60 * 10),
                      int(wind_node.getFloat("wind_dir_deg") * 100),
                      int(wind_node.getFloat("wind_speed_kt") * 4),
                      int(wind_node.getFloat("pitot_scale_factor") * 100),
                      node.getInt("error_count"),
                      node.getInt("status"))
    return wrap_packet(AIRDATA_PACKET_V7, buf)

def pack_airdata_dict(index):
    airdata_node = getNode('/sensors/airdata[%d]' % index, True)
    row = dict()
    row['timestamp'] = airdata_node.getFloat('timestamp')
    row['pressure_mbar'] = airdata_node.getFloat('pressure_mbar')
    row['temp_C'] = airdata_node.getFloat('temp_C')
    row['airspeed_smoothed_kt'] = vel_node.getFloat('airspeed_smoothed_kt')
    row['altitude_smoothed_m'] = pos_pressure_node.getFloat('altitude_smoothed_m')
    row['altitude_true_m'] = pos_combined_node.getFloat('altitude_true_m')
    row['wind_dir_deg'] = wind_node.getFloat('wind_dir_deg')
    row['wind_speed_kt'] = wind_node.getFloat('wind_speed_kt')
    row['pitot_scale_factor'] = wind_node.getFloat('pitot_scale_factor')
    row['tecs_error_total'] = tecs_node.getFloat('error_total')
    row['tecs_error_diff'] = tecs_node.getFloat('error_diff')
    row['status'] = airdata_node.getInt('status')
    return row

def pack_airdata_csv(index):
    airdata_node = getNode('/sensors/airdata[%d]' % index, True)
    row = dict()
    row['timestamp'] = '%.4f' % airdata_node.getFloat('timestamp')
    row['pressure_mbar'] = '%.1f' % airdata_node.getFloat('pressure_mbar')
    row['temp_C'] = '%.1f' % airdata_node.getFloat('temp_C')
    row['airspeed_smoothed_kt'] = '%.1f' % vel_node.getFloat('airspeed_smoothed_kt')
    row['altitude_smoothed_m'] = '%.2f' % pos_pressure_node.getFloat('altitude_smoothed_m')
    row['altitude_true_m'] = '%.2f' % pos_combined_node.getFloat('altitude_true_m')
    row['wind_dir_deg'] = '%.1f' % wind_node.getFloat('wind_dir_deg')
    row['wind_speed_kt'] = '%.1f' % wind_node.getFloat('wind_speed_kt')
    row['pitot_scale_factor'] = '%.2f' % wind_node.getFloat('pitot_scale_factor')
    row['tecs_error_total'] = '%.2f' % tecs_node.getFloat('error_total')
    row['tecs_error_diff'] = '%.2f' % tecs_node.getFloat('error_diff')
    row['status'] = '%d' % airdata_node.getInt('status')
    keys = ['timestamp', 'pressure_mbar', 'temp_C', 'airspeed_smoothed_kt',
            'altitude_smoothed_m', 'altitude_true_m',
            'wind_dir_deg', 'wind_speed_kt', 'pitot_scale_factor',
            'tecs_error_total', 'tecs_error_diff', 'status']
    return row, keys

def unpack_airdata_v5(buf):
    result = struct.unpack(airdata_v5_fmt, buf)
    
    index = result[0]
    if index >= len(airdata_nodes):
        for i in range(len(airdata_nodes),index+1):
            path = '/sensors/airdata[%d]' % i
            airdata_nodes.append( getNode(path, True) )
    node = airdata_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setFloat("pressure_mbar", result[2] / 10.0)
    node.setFloat("temp_C", result[3] / 100.0)
    vel_node.setFloat("airspeed_smoothed_kt", result[4] / 100.0)
    pos_pressure_node.setFloat("altitude_smoothed_m", result[5])
    pos_combined_node.setFloat("altitude_true_m", result[6])
    vel_node.setFloat("pressure_vertical_speed_fps", (result[7] / 10.0) / 60.0)
    wind_node.setFloat("wind_dir_deg", result[8] / 100.0)
    wind_node.setFloat("wind_speed_kt", result[9] / 4.0)
    wind_node.setFloat("pitot_scale_factor", result[10] / 100.0)
    node.setInt("status", result[11])

    return index

def unpack_airdata_v6(buf):
    result = struct.unpack(airdata_v6_fmt, buf)
    
    index = result[0]
    if index >= len(airdata_nodes):
        for i in range(len(airdata_nodes),index+1):
            path = '/sensors/airdata[%d]' % i
            airdata_nodes.append( getNode(path, True) )
    node = airdata_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setFloat("pressure_mbar", result[2] / 10.0)
    node.setFloat("temp_C", result[3] / 100.0)
    vel_node.setFloat("airspeed_smoothed_kt", result[4] / 100.0)
    pos_pressure_node.setFloat("altitude_smoothed_m", result[5])
    pos_combined_node.setFloat("altitude_true_m", result[6])
    vel_node.setFloat("pressure_vertical_speed_fps", (result[7] / 10.0) / 60.0)
    wind_node.setFloat("wind_dir_deg", result[8] / 100.0)
    wind_node.setFloat("wind_speed_kt", result[9] / 4.0)
    wind_node.setFloat("pitot_scale_factor", result[10] / 100.0)
    node.setInt("status", result[11])

    return index

def unpack_airdata_v7(buf):
    result = struct.unpack(airdata_v7_fmt, buf)
    
    index = result[0]
    if index >= len(airdata_nodes):
        for i in range(len(airdata_nodes),index+1):
            path = '/sensors/airdata[%d]' % i
            airdata_nodes.append( getNode(path, True) )
    node = airdata_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setFloat("pressure_mbar", result[2] / 10.0)
    node.setFloat("temp_C", result[3] / 100.0)
    vel_node.setFloat("airspeed_smoothed_kt", result[4] / 100.0)
    pos_pressure_node.setFloat("altitude_smoothed_m", result[5])
    pos_combined_node.setFloat("altitude_true_m", result[6])
    vel_node.setFloat("pressure_vertical_speed_fps", (result[7] / 10.0) / 60.0)
    wind_node.setFloat("wind_dir_deg", result[8] / 100.0)
    wind_node.setFloat("wind_speed_kt", result[9] / 4.0)
    wind_node.setFloat("pitot_scale_factor", result[10] / 100.0)
    node.setInt("error_count", result[11])
    node.setInt("status", result[12])

    return index

def pack_filter_bin(index):
    if index >= len(filter_nodes):
        for i in range(len(filter_nodes),index+1):
            path = '/filters/filter[%d]' % i
            filter_nodes.append( getNode(path, True) )
    node = filter_nodes[index]

    buf = struct.pack(filter_v4_fmt,
                      index,
                      node.getFloat("timestamp"),
                      node.getFloat("latitude_deg"),
                      node.getFloat("longitude_deg"),
                      node.getFloat("altitude_m"),
                      int(node.getFloat("vn_ms") * 100),
                      int(node.getFloat("ve_ms") * 100),
                      int(node.getFloat("vd_ms") * 100),
                      int(node.getFloat("roll_deg") * 10),
                      int(node.getFloat("pitch_deg") * 10),
                      int(node.getFloat("heading_deg") * 10),
                      int(round(node.getFloat("p_bias") * 10000.0)),
                      int(round(node.getFloat("q_bias") * 10000.0)),
                      int(round(node.getFloat("r_bias") * 10000.0)),
                      int(round(node.getFloat("ax_bias") * 1000.0)),
                      int(round(node.getFloat("ay_bias") * 1000.0)),
                      int(round(node.getFloat("az_bias") * 1000.0)),
                      remote_link_node.getInt("sequence_num"),
                      0)
    return wrap_packet(FILTER_PACKET_V4, buf)

def pack_filter_dict(index):
    filter_node = getNode('/filters/filter[%d]' % index, True)
    row = dict()
    row['timestamp'] = filter_node.getFloat('timestamp')
    row['latitude_deg'] = filter_node.getFloat('latitude_deg')
    row['longitude_deg'] = filter_node.getFloat('longitude_deg')
    row['altitude_m'] = filter_node.getFloat('altitude_m')
    row['vn_ms'] = filter_node.getFloat('vn_ms')
    row['ve_ms'] = filter_node.getFloat('ve_ms')
    row['vd_ms'] = filter_node.getFloat('vd_ms')
    row['roll_deg'] = filter_node.getFloat('roll_deg')
    row['pitch_deg'] = filter_node.getFloat('pitch_deg')
    row['heading_deg'] = filter_node.getFloat('heading_deg')
    row['p_bias'] = filter_node.getFloat('p_bias')
    row['q_bias'] = filter_node.getFloat('q_bias')
    row['r_bias'] = filter_node.getFloat('r_bias')
    row['ax_bias'] = filter_node.getFloat('ax_bias')
    row['ay_bias'] = filter_node.getFloat('ay_bias')
    row['az_bias'] = filter_node.getFloat('az_bias')
    row['status'] = filter_node.getInt('status')
    return row

def pack_filter_csv(index):
    filter_node = getNode('/filters/filter[%d]' % index, True)
    row = dict()
    row['timestamp'] = '%.4f' % filter_node.getFloat('timestamp')
    row['latitude_deg'] = '%.10f' % filter_node.getFloat('latitude_deg')
    row['longitude_deg'] = '%.10f' % filter_node.getFloat('longitude_deg')
    row['altitude_m'] = '%.2f' % filter_node.getFloat('altitude_m')
    row['vn_ms'] = '%.4f' % filter_node.getFloat('vn_ms')
    row['ve_ms'] = '%.4f' % filter_node.getFloat('ve_ms')
    row['vd_ms'] = '%.4f' % filter_node.getFloat('vd_ms')
    row['roll_deg'] = '%.3f' % filter_node.getFloat('roll_deg')
    row['pitch_deg'] = '%.3f' % filter_node.getFloat('pitch_deg')
    row['heading_deg'] = '%.3f' % filter_node.getFloat('heading_deg')
    row['p_bias'] = '%.4f' % filter_node.getFloat('p_bias')
    row['q_bias'] = '%.4f' % filter_node.getFloat('q_bias')
    row['r_bias'] = '%.4f' % filter_node.getFloat('r_bias')
    row['ax_bias'] = '%.3f' % filter_node.getFloat('ax_bias')
    row['ay_bias'] = '%.3f' % filter_node.getFloat('ay_bias')
    row['az_bias'] = '%.3f' % filter_node.getFloat('az_bias')
    row['status'] = '%d' % filter_node.getInt('status')
    keys = ['timestamp', 'latitude_deg', 'longitude_deg', 'altitude_m',
            'vn_ms', 've_ms', 'vd_ms', 'roll_deg', 'pitch_deg', 'heading_deg',
            'p_bias', 'q_bias', 'r_bias', 'ax_bias', 'ay_bias', 'az_bias',
            'status']
    return row, keys

def unpack_filter_v2(buf):
    result = struct.unpack(filter_v2_fmt, buf)
    
    index = result[0]
    if index >= len(filter_nodes):
        for i in range(len(filter_nodes),index+1):
            path = '/filters/filter[%d]' % i
            filter_nodes.append( getNode(path, True) )
    node = filter_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setFloat("latitude_deg", result[2])
    node.setFloat("longitude_deg", result[3])
    node.setFloat("altitude_m", result[4])
    node.setFloat("vn_ms", result[5] / 100.0)
    node.setFloat("ve_ms", result[6] / 100.0)
    node.setFloat("vd_ms", result[7] / 100.0)
    node.setFloat("roll_deg", result[8] / 10.0)
    node.setFloat("pitch_deg", result[9] / 10.0)
    node.setFloat("heading_deg", result[10] / 10.0)
    remote_link_node.setInt("sequence_num", result[11])
    node.setInt("status", result[12])

    return index

def unpack_filter_v3(buf):
    result = struct.unpack(filter_v3_fmt, buf)
    
    index = result[0]
    if index >= len(filter_nodes):
        for i in range(len(filter_nodes),index+1):
            path = '/filters/filter[%d]' % i
            filter_nodes.append( getNode(path, True) )
    node = filter_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setFloat("latitude_deg", result[2])
    node.setFloat("longitude_deg", result[3])
    node.setFloat("altitude_m", result[4])
    node.setFloat("vn_ms", result[5] / 100.0)
    node.setFloat("ve_ms", result[6] / 100.0)
    node.setFloat("vd_ms", result[7] / 100.0)
    node.setFloat("roll_deg", result[8] / 10.0)
    node.setFloat("pitch_deg", result[9] / 10.0)
    node.setFloat("heading_deg", result[10] / 10.0)
    node.setFloat("p_bias", result[11] / 10000.0)
    node.setFloat("q_bias", result[12] / 10000.0)
    node.setFloat("r_bias", result[13] / 10000.0)
    node.setFloat("ax_bias", result[14] / 1000.0)
    node.setFloat("ay_bias", result[15] / 1000.0)
    node.setFloat("az_bias", result[16] / 1000.0)
    remote_link_node.setInt("sequence_num", result[17])
    node.setInt("status", result[18])

    return index

def unpack_filter_v4(buf):
    result = struct.unpack(filter_v4_fmt, buf)
    
    index = result[0]
    if index >= len(filter_nodes):
        for i in range(len(filter_nodes),index+1):
            path = '/filters/filter[%d]' % i
            filter_nodes.append( getNode(path, True) )
    node = filter_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setFloat("latitude_deg", result[2])
    node.setFloat("longitude_deg", result[3])
    node.setFloat("altitude_m", result[4])
    node.setFloat("vn_ms", result[5] / 100.0)
    node.setFloat("ve_ms", result[6] / 100.0)
    node.setFloat("vd_ms", result[7] / 100.0)
    node.setFloat("roll_deg", result[8] / 10.0)
    node.setFloat("pitch_deg", result[9] / 10.0)
    node.setFloat("heading_deg", result[10] / 10.0)
    node.setFloat("p_bias", result[11] / 10000.0)
    node.setFloat("q_bias", result[12] / 10000.0)
    node.setFloat("r_bias", result[13] / 10000.0)
    node.setFloat("ax_bias", result[14] / 1000.0)
    node.setFloat("ay_bias", result[15] / 1000.0)
    node.setFloat("az_bias", result[16] / 1000.0)
    remote_link_node.setInt("sequence_num", result[17])
    node.setInt("status", result[18])

    return index

def pack_act_bin(index):
    if index > 0:
        return
    buf = struct.pack(act_v3_fmt,
                      0, # always zero for now
                      act_node.getFloat("timestamp"),
                      int(act_node.getFloat("aileron") * 20000),
                      int(act_node.getFloat("elevator") * 20000),
                      int(act_node.getFloat("throttle") * 60000),
                      int(act_node.getFloat("rudder") * 20000),
                      int(act_node.getFloat("channel5") * 20000),
                      int(act_node.getFloat("flaps") * 20000),
                      int(act_node.getFloat("channel7") * 20000),
                      int(act_node.getFloat("channel8") * 20000),
                      0)
    return wrap_packet(ACTUATOR_PACKET_V3, buf)

def pack_act_dict(index):
    row = dict()
    row['timestamp'] = act_node.getFloat('timestamp')
    row['aileron_norm'] = act_node.getFloat('aileron')
    row['elevator_norm'] = act_node.getFloat('elevator')
    row['throttle_norm'] = act_node.getFloat('throttle')
    row['rudder_norm'] = act_node.getFloat('rudder')
    row['channel5_norm'] = act_node.getFloat('channel5')
    row['flaps_norm'] = act_node.getFloat('flaps')
    row['channel7_norm'] = act_node.getFloat('channel7')
    row['channel8_norm'] = act_node.getFloat('channel8')
    row['status'] = act_node.getInt('status')
    return row

def pack_act_csv(index):
    row = dict()
    row['timestamp'] = '%.4f' % act_node.getFloat('timestamp')
    row['aileron_norm'] = '%.4f' % act_node.getFloat('aileron')
    row['elevator_norm'] = '%.4f' % act_node.getFloat('elevator')
    row['throttle_norm'] = '%.4f' % act_node.getFloat('throttle')
    row['rudder_norm'] = '%.4f' % act_node.getFloat('rudder')
    row['channel5_norm'] = '%.4f' % act_node.getFloat('channel5')
    row['flaps_norm'] = '%.4f' % act_node.getFloat('flaps')
    row['channel7_norm'] = '%.4f' % act_node.getFloat('channel7')
    row['channel8_norm'] = '%.4f' % act_node.getFloat('channel8')
    row['status'] = '%d' % act_node.getInt('status')
    keys = ['timestamp', 'aileron_norm', 'elevator_norm', 'throttle_norm',
            'rudder_norm', 'channel5_norm', 'flaps_norm', 'channel7_norm',
            'channel8_norm', 'status']
    return row, keys

def unpack_act_v2(buf):
    result = struct.unpack(act_v2_fmt, buf)
    index = result[0]
    act_node.setFloat("timestamp", result[1])
    act_node.setFloat("aileron", result[2] / 20000.0)
    act_node.setFloat("elevator", result[3] / 20000.0)
    act_node.setFloat("throttle", result[4] / 60000.0)
    act_node.setFloat("rudder", result[5] / 20000.0)
    act_node.setFloat("channel5", result[6] / 20000.0)
    act_node.setFloat("flaps", result[7] / 20000.0)
    act_node.setFloat("channel7", result[8] / 20000.0)
    act_node.setFloat("channel8", result[9] / 20000.0)
    act_node.setInt("status", result[10])

    return index

def unpack_act_v3(buf):
    result = struct.unpack(act_v3_fmt, buf)
    index = result[0]
    act_node.setFloat("timestamp", result[1])
    act_node.setFloat("aileron", result[2] / 20000.0)
    act_node.setFloat("elevator", result[3] / 20000.0)
    act_node.setFloat("throttle", result[4] / 60000.0)
    act_node.setFloat("rudder", result[5] / 20000.0)
    act_node.setFloat("channel5", result[6] / 20000.0)
    act_node.setFloat("flaps", result[7] / 20000.0)
    act_node.setFloat("channel7", result[8] / 20000.0)
    act_node.setFloat("channel8", result[9] / 20000.0)
    act_node.setInt("status", result[10])

    return index

def pack_pilot_bin(index):
    if index >= len(pilot_nodes):
        for i in range(len(pilot_nodes),index+1):
            path = '/sensors/pilot_input[%d]' % i
            node = getNode(path, True)
            node.setLen("channel", NUM_ACTUATORS, 0.0)
            pilot_nodes.append( node )
    node = pilot_nodes[index]
    
    buf = struct.pack(pilot_v3_fmt,
                      index,
                      node.getFloat("timestamp"),
                      int(node.getFloatEnum("channel", 0) * 20000),
                      int(node.getFloatEnum("channel", 1) * 20000),
                      int(node.getFloatEnum("channel", 2) * 20000),
                      int(node.getFloatEnum("channel", 3) * 20000),
                      int(node.getFloatEnum("channel", 4) * 20000),
                      int(node.getFloatEnum("channel", 5) * 20000),
                      int(node.getFloatEnum("channel", 6) * 20000),
                      int(node.getFloatEnum("channel", 7) * 20000),
                      0)
    return wrap_packet(PILOT_INPUT_PACKET_V3, buf)

def pack_pilot_dict(index):
    pilot_node = getNode('/sensors/pilot_input[%d]' % index, True)
    row = dict()
    row['timestamp'] = pilot_node.getFloat('timestamp')
    row['channel[0]'] = pilot_node.getFloatEnum('channel', 0)
    row['channel[1]'] = pilot_node.getFloatEnum('channel', 1)
    row['channel[2]'] = pilot_node.getFloatEnum('channel', 2)
    row['channel[3]'] = pilot_node.getFloatEnum('channel', 3)
    row['channel[4]'] = pilot_node.getFloatEnum('channel', 4)
    row['channel[5]'] = pilot_node.getFloatEnum('channel', 5)
    row['channel[6]'] = pilot_node.getFloatEnum('channel', 6)
    row['channel[7]'] = pilot_node.getFloatEnum('channel', 7)
    row['status'] = pilot_node.getInt('status')
    return row

def pack_pilot_csv(index):
    pilot_node = getNode('/sensors/pilot_input[%d]' % index, True)
    row = dict()
    row['timestamp'] = '%.4f' % pilot_node.getFloat('timestamp')
    row['channel[0]'] = '%.3f' % pilot_node.getFloatEnum('channel', 0)
    row['channel[1]'] = '%.3f' % pilot_node.getFloatEnum('channel', 1)
    row['channel[2]'] = '%.3f' % pilot_node.getFloatEnum('channel', 2)
    row['channel[3]'] = '%.3f' % pilot_node.getFloatEnum('channel', 3)
    row['channel[4]'] = '%.3f' % pilot_node.getFloatEnum('channel', 4)
    row['channel[5]'] = '%.3f' % pilot_node.getFloatEnum('channel', 5)
    row['channel[6]'] = '%.3f' % pilot_node.getFloatEnum('channel', 6)
    row['channel[7]'] = '%.3f' % pilot_node.getFloatEnum('channel', 7)
    row['status'] = '%d' % pilot_node.getInt('status')
    keys = ['timestamp', 'channel[0]', 'channel[1]', 'channel[2]',
            'channel[3]', 'channel[4]', 'channel[5]', 'channel[6]',
            'channel[7]', 'status']
    return row, keys

def unpack_pilot_v2(buf):
    result = struct.unpack(pilot_v2_fmt, buf)
    
    index = result[0]
    if index >= len(pilot_nodes):
        for i in range(len(pilot_nodes),index+1):
            node = getNode('/sensors/pilot_input[%d]' % i, True)
            node.setLen("channel", NUM_ACTUATORS, 0.0)
            pilot_nodes.append( node )
    node = pilot_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setFloatEnum("channel", 0, result[2] / 20000.0)
    node.setFloatEnum("channel", 1, result[3] / 20000.0)
    node.setFloatEnum("channel", 2, result[4] / 20000.0)
    node.setFloatEnum("channel", 3, result[5] / 20000.0)
    node.setFloatEnum("channel", 4, result[6] / 20000.0)
    node.setFloatEnum("channel", 5, result[7] / 20000.0)
    node.setFloatEnum("channel", 6, result[8] / 20000.0)
    node.setFloatEnum("channel", 7, result[9] / 20000.0)
    node.setInt("status", result[10])

    return index

def unpack_pilot_v3(buf):
    result = struct.unpack(pilot_v3_fmt, buf)
    
    index = result[0]
    if index >= len(pilot_nodes):
        for i in range(len(pilot_nodes),index+1):
            node = getNode('/sensors/pilot_input[%d]' % i, True)
            node.setLen("channel", NUM_ACTUATORS, 0.0)
            pilot_nodes.append( node )
    node = pilot_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setFloatEnum("channel", 0, result[2] / 20000.0)
    node.setFloatEnum("channel", 1, result[3] / 20000.0)
    node.setFloatEnum("channel", 2, result[4] / 20000.0)
    node.setFloatEnum("channel", 3, result[5] / 20000.0)
    node.setFloatEnum("channel", 4, result[6] / 20000.0)
    node.setFloatEnum("channel", 5, result[7] / 20000.0)
    node.setFloatEnum("channel", 6, result[8] / 20000.0)
    node.setFloatEnum("channel", 7, result[9] / 20000.0)
    node.setInt("status", result[10])

    return index

def pack_ap_status_bin(index):
    # status flags (up to 8 could be supported)
    flags = 0
    if ap_node.getBool("master_switch"):
        flags |= (1 << 0)
    if ap_node.getBool("pilot_pass_through"):
        flags |= (1 << 1)
    
    # handle the counter dance between the control module and the
    # packer.  This allows us to trickle down routes to the ground
    # station, but we don't want onboard logging to affect the counter
    # state.
    counter = remote_link_node.getInt("wp_counter")
    route_size = active_node.getInt("route_size")
    if counter >= route_size + 2:
        counter = 0
        remote_link_node.setInt("wp_counter", 0)

    target_agl_ft = targets_node.getFloat("altitude_agl_ft")
    ground_m = pos_node.getFloat("altitude_ground_m")
    # if pressure based ...
    #   error_m = pos_pressure_node.getFloat("pressure_error_m")
    #   target_msl_ft = (ground_m + error_m) * m2ft + target_agl_ft
    # else:
    target_msl_ft = ground_m * m2ft + target_agl_ft

    wp_lon = 0.0
    wp_lat = 0.0
    task_attr = 0
    if route_size > 0 and counter < route_size:
        wp_index = counter
        wp_node = active_node.getChild('wpt[%d]' % wp_index)
        if wp_node != None:
            wp_lon = wp_node.getFloat("longitude_deg")
            wp_lat = wp_node.getFloat("latitude_deg")
    elif counter == route_size:
        wp_lon = circle_node.getFloat("longitude_deg")
        wp_lat = circle_node.getFloat("latitude_deg")
        wp_index = 65534
        task_attr = int(round(circle_node.getFloat("radius_m") * 10))
        if task_attr > 32767: task_attr = 32767
    elif counter == route_size + 1:
        wp_lon = home_node.getFloat("longitude_deg")
        wp_lat = home_node.getFloat("latitude_deg")
        wp_index = 65535
    
    task_id = 0                 # code for unknown or not set
    if task_node.getString("current_task_id") == 'circle':
        task_id = 1
    elif task_node.getString("current_task_id") == 'parametric':
        # draw like it's a circle
        task_id = 1
    elif task_node.getString("current_task_id") == 'route':
        task_id = 2
    elif task_node.getString("current_task_id") == 'land':
        task_id = 3

    #print index,                   status_node.getFloat('frame_time'),                      int(targets_node.getFloat("groundtrack_deg") * 10),                      int(targets_node.getFloat("roll_deg") * 10),                      int(target_msl_ft),                      int(targets_node.getFloat("climb_rate_fps") * 10),                      int(targets_node.getFloat("pitch_deg") * 10),                      int(targets_node.getFloat("the_dot") * 1000),                      int(targets_node.getFloat("airspeed_kt") * 10),                      int(task_node.getFloat("flight_timer")),                      route_node.getInt("target_waypoint_idx"),                      wp_lon,                      wp_lat,                      wp_index,                      route_size,                      remote_link_node.getInt("sequence_num")
    buf = struct.pack(ap_status_v7_fmt,
                      index,
                      status_node.getFloat('frame_time'),
                      flags,
                      int(round(targets_node.getFloat("groundtrack_deg") * 10)),
                      int(round(targets_node.getFloat("roll_deg") * 10)),
                      int(round(target_msl_ft)),
                      int(round(ground_m)),
                      int(round(targets_node.getFloat("pitch_deg") * 10)),
                      int(round(targets_node.getFloat("airspeed_kt") * 10)),
                      int(round(task_node.getFloat("flight_timer"))) % 64800,
                      route_node.getInt("target_waypoint_idx"),
                      wp_lon,
                      wp_lat,
                      wp_index,
                      route_size,
                      task_id,
                      task_attr,
                      remote_link_node.getInt("sequence_num"))
    # print 'B:', index, flags, remote_link_node.getInt("sequence_num")
    # print 'd:', status_node.getFloat('frame_time'), wp_lon, wp_lat
    # print 'h:', int(round(targets_node.getFloat("groundtrack_deg") * 10)), int(round(targets_node.getFloat("roll_deg") * 10)),
    # print 'H:', int(round(target_msl_ft)), int(round(ground_m)),
    # int(round(targets_node.getFloat("pitch_deg") * 10)), int(round(targets_node.getFloat("airspeed_kt") * 10)), int(round(task_node.getFloat("flight_timer"))), route_node.getInt("target_waypoint_idx"), wp_index, route_size, 
    return wrap_packet(AP_STATUS_PACKET_V7, buf)

def pack_ap_status_dict(index):
    # fixme: tecs_target_tot is really zero now because these values
    # are computed in energy *error* terms
    row = dict()
    row['timestamp'] = targets_node.getFloat('timestamp')
    row['master_switch'] = ap_node.getBool("master_switch")
    row['pilot_pass_through'] = ap_node.getBool("pilot_pass_through")
    row['groundtrack_deg'] = targets_node.getFloat('groundtrack_deg')
    row['roll_deg'] = targets_node.getFloat('roll_deg')
    row['altitude_msl_ft'] = targets_node.getFloat('altitude_msl_ft')
    row['pitch_deg'] = targets_node.getFloat('pitch_deg')
    row['airspeed_kt'] = targets_node.getFloat('airspeed_kt')
    row['altitude_ground_m'] = pos_node.getFloat("altitude_ground_m")
    row['tecs_target_tot'] = tecs_node.getFloat("target_total")
    return row

def pack_ap_status_csv(index):
    # fixme: tecs_target_tot is really zero now because these values
    # are computed in energy *error* terms
    row = dict()
    row['timestamp'] = '%.4f' % targets_node.getFloat('timestamp')
    row['master_switch'] = '%d' % ap_node.getBool("master_switch")
    row['pilot_pass_through'] = '%d' % ap_node.getBool("pilot_pass_through")
    row['groundtrack_deg'] = '%.2f' % targets_node.getFloat('groundtrack_deg')
    row['roll_deg'] = '%.2f' % targets_node.getFloat('roll_deg')
    row['altitude_msl_ft'] = '%.2f' % targets_node.getFloat('altitude_msl_ft')
    row['pitch_deg'] = '%.2f' % targets_node.getFloat('pitch_deg')
    row['airspeed_kt'] = '%.1f' % targets_node.getFloat('airspeed_kt')
    row['altitude_ground_m'] = '%.1f' % pos_node.getFloat("altitude_ground_m")
    row['tecs_target_tot'] = '%.4f' % tecs_node.getFloat("target_total")
    keys = ['timestamp', 'master_switch', 'pilot_pass_through',
            'groundtrack_deg', 'roll_deg', 'altitude_msl_ft', 'pitch_deg',
            'airspeed_kt', 'altitude_ground_m',
            'tecs_target_tot']
    return row, keys

def unpack_ap_status_v4(buf):
    result = struct.unpack(ap_status_v4_fmt, buf)

    index = result[0]

    wp_lon = result[10]
    wp_lat = result[11]
    wp_index = result[12]
    route_size = result[13]
    
    targets_node.setFloat("timestamp", result[1])
    targets_node.setFloat("groundtrack_deg", result[2] / 10.0)
    targets_node.setFloat("roll_deg", result[3] / 10.0)
    targets_node.setFloat("altitude_msl_ft", result[4])
    pos_node.setFloat("altitude_ground_m", result[5])
    targets_node.setFloat("pitch_deg", result[6] / 10.0)
    targets_node.setFloat("airspeed_kt", result[7] / 10.0)
    status_node.setFloat("flight_timer", result[8])
    route_node.setInt("target_waypoint_idx", result[9])
    if wp_index < route_size:
        wp_node = active_node.getChild('wpt[%d]' % wp_index, True)
        wp_node.setFloat("longitude_deg", wp_lon)
        wp_node.setFloat("latitude_deg", wp_lat)
    elif wp_index == 65534:
        circle_node.setFloat("longitude_deg", wp_lon)
        circle_node.setFloat("latitude_deg", wp_lat)
    elif wp_index == 65535:
        home_node.setFloat("longitude_deg", wp_lon)
        home_node.setFloat("latitude_deg", wp_lat)
    active_node.setInt("route_size", route_size)
    remote_link_node.setInt("sequence_num", result[14])

    return index

def unpack_ap_status_v5(buf):
    result = struct.unpack(ap_status_v5_fmt, buf)

    index = result[0]

    wp_lon = result[11]
    wp_lat = result[12]
    wp_index = result[13]
    route_size = result[14]
    
    targets_node.setFloat("timestamp", result[1])
    flags = result[2]
    ap_node.setBool("master_switch", flags & (1<<0))
    ap_node.setBool("pilot_pass_through", flags & (1<<1))
    targets_node.setFloat("groundtrack_deg", result[3] / 10.0)
    targets_node.setFloat("roll_deg", result[4] / 10.0)
    targets_node.setFloat("altitude_msl_ft", result[5])
    pos_node.setFloat("altitude_ground_m", result[6])
    targets_node.setFloat("pitch_deg", result[7] / 10.0)
    targets_node.setFloat("airspeed_kt", result[8] / 10.0)
    status_node.setFloat("flight_timer", result[9])
    route_node.setInt("target_waypoint_idx", result[10])
    if wp_index < route_size:
        wp_node = active_node.getChild('wpt[%d]' % wp_index, True)
        wp_node.setFloat("longitude_deg", wp_lon)
        wp_node.setFloat("latitude_deg", wp_lat)
    elif wp_index == 65534:
        circle_node.setFloat("longitude_deg", wp_lon)
        circle_node.setFloat("latitude_deg", wp_lat)
    elif wp_index == 65535:
        home_node.setFloat("longitude_deg", wp_lon)
        home_node.setFloat("latitude_deg", wp_lat)
    active_node.setInt("route_size", route_size)
    remote_link_node.setInt("sequence_num", result[15])

    return index

def unpack_ap_status_v6(buf):
    result = struct.unpack(ap_status_v6_fmt, buf)

    index = result[0]

    wp_lon = result[11]
    wp_lat = result[12]
    wp_index = result[13]
    route_size = result[14]
    task_id = result[15]
    task_attrib = result[16]
    
    targets_node.setFloat("timestamp", result[1])
    flags = result[2]
    ap_node.setBool("master_switch", flags & (1<<0))
    ap_node.setBool("pilot_pass_through", flags & (1<<1))
    targets_node.setFloat("groundtrack_deg", result[3] / 10.0)
    targets_node.setFloat("roll_deg", result[4] / 10.0)
    targets_node.setFloat("altitude_msl_ft", result[5])
    pos_node.setFloat("altitude_ground_m", result[6])
    targets_node.setFloat("pitch_deg", result[7] / 10.0)
    targets_node.setFloat("airspeed_kt", result[8] / 10.0)
    status_node.setFloat("flight_timer", result[9])
    if route_size != active_node.getInt("route_size"):
        # route size change, zero all the waypoint coordinates
        for i in range(active_node.getInt("route_size")):
            wp_node = active_node.getChild('wpt[%d]' % i, True)
            wp_node.setFloat("longitude_deg", 0)
            wp_node.setFloat("latitude_deg", 0)
    route_node.setInt("target_waypoint_idx", result[10])
    if wp_index < route_size:
        wp_node = active_node.getChild('wpt[%d]' % wp_index, True)
        wp_node.setFloat("longitude_deg", wp_lon)
        wp_node.setFloat("latitude_deg", wp_lat)
    elif wp_index == 65534:
        circle_node.setFloat("longitude_deg", wp_lon)
        circle_node.setFloat("latitude_deg", wp_lat)
        circle_node.setFloat("radius_m", task_attrib / 10.0)
    elif wp_index == 65535:
        home_node.setFloat("longitude_deg", wp_lon)
        home_node.setFloat("latitude_deg", wp_lat)
    if task_id == 1:
        task_node.setString("current_task_id", "circle");
    elif task_id == 2:
        task_node.setString("current_task_id", "route");
    elif task_id == 3:
        task_node.setString("current_task_id", "land");
    else:
        task_node.setString("current_task_id", "unknown");
                
    active_node.setInt("route_size", route_size)
    remote_link_node.setInt("sequence_num", result[17])

    return index

def unpack_ap_status_v7(buf):
    result = struct.unpack(ap_status_v7_fmt, buf)

    index = result[0]

    wp_lon = result[11]
    wp_lat = result[12]
    wp_index = result[13]
    route_size = result[14]
    task_id = result[15]
    task_attrib = result[16]
    
    targets_node.setFloat("timestamp", result[1])
    flags = result[2]
    ap_node.setBool("master_switch", flags & (1<<0))
    ap_node.setBool("pilot_pass_through", flags & (1<<1))
    targets_node.setFloat("groundtrack_deg", result[3] / 10.0)
    targets_node.setFloat("roll_deg", result[4] / 10.0)
    targets_node.setFloat("altitude_msl_ft", result[5])
    pos_node.setFloat("altitude_ground_m", result[6])
    targets_node.setFloat("pitch_deg", result[7] / 10.0)
    targets_node.setFloat("airspeed_kt", result[8] / 10.0)
    status_node.setFloat("flight_timer", result[9])
    if route_size != active_node.getInt("route_size"):
        # route size change, zero all the waypoint coordinates
        for i in range(active_node.getInt("route_size")):
            wp_node = active_node.getChild('wpt[%d]' % i, True)
            wp_node.setFloat("longitude_deg", 0)
            wp_node.setFloat("latitude_deg", 0)
    route_node.setInt("target_waypoint_idx", result[10])
    if wp_index < route_size:
        wp_node = active_node.getChild('wpt[%d]' % wp_index, True)
        wp_node.setFloat("longitude_deg", wp_lon)
        wp_node.setFloat("latitude_deg", wp_lat)
    elif wp_index == 65534:
        circle_node.setFloat("longitude_deg", wp_lon)
        circle_node.setFloat("latitude_deg", wp_lat)
        circle_node.setFloat("radius_m", task_attrib / 10.0)
    elif wp_index == 65535:
        home_node.setFloat("longitude_deg", wp_lon)
        home_node.setFloat("latitude_deg", wp_lat)
    if task_id == 1:
        task_node.setString("current_task_id", "circle");
    elif task_id == 2:
        task_node.setString("current_task_id", "route");
    elif task_id == 3:
        task_node.setString("current_task_id", "land");
    else:
        task_node.setString("current_task_id", "unknown");
                
    active_node.setInt("route_size", route_size)
    remote_link_node.setInt("sequence_num", result[17])

    return index

def pack_system_health_bin(index):
    dekamah = int(power_node.getFloat("total_mah") / 10)
    if dekamah < 0: dekamah = 0 # prevent overflowing the structure
    if dekamah > 65535: dekamah = 65535 # prevent overflowing the structure
    buf = struct.pack(system_health_v5_fmt,
                      index,
                      status_node.getFloat('frame_time'),
                      int(status_node.getFloat("system_load_avg") * 100),
                      int(power_node.getFloat("avionics_vcc") * 1000),
                      int(power_node.getFloat("main_vcc") * 1000),
                      int(power_node.getFloat("cell_vcc") * 1000),
                      int(power_node.getFloat("main_amps") * 1000),
                      dekamah)
    return wrap_packet(SYSTEM_HEALTH_PACKET_V5, buf)

def pack_system_health_dict(index):
    row = dict()
    row['timestamp'] = status_node.getFloat('frame_time')
    row['system_load_avg'] = status_node.getFloat('system_load_avg')
    row['avionics_vcc'] = power_node.getFloat('avionics_vcc')
    row['main_vcc'] = power_node.getFloat('main_vcc')
    row['cell_vcc'] = power_node.getFloat('cell_vcc')
    row['main_amps'] = power_node.getFloat('main_amps')
    row['total_mah'] = power_node.getFloat('total_mah')
    return row

def pack_system_health_csv(index):
    row = dict()
    row['timestamp'] = '%.4f' % status_node.getFloat('frame_time')
    row['system_load_avg'] = '%.2f' % status_node.getFloat('system_load_avg')
    row['avionics_vcc'] = '%.2f' % power_node.getFloat('avionics_vcc')
    row['main_vcc'] = '%.2f' % power_node.getFloat('main_vcc')
    row['cell_vcc'] = '%.2f' % power_node.getFloat('cell_vcc')
    row['main_amps'] = '%.2f' % power_node.getFloat('main_amps')
    row['total_mah'] = '%.0f' % power_node.getFloat('total_mah')
    keys = ['timestamp', 'system_load_avg', 'avionics_vcc', 'main_vcc',
            'cell_vcc', 'main_amps', 'total_mah']
    return row, keys

def unpack_system_health_v4(buf):
    result = struct.unpack(system_health_v4_fmt, buf)

    index = result[0]
    
    status_node.setFloat("frame_time", result[1])
    status_node.setFloat("system_load_avg", result[2] / 100.0)
    power_node.setFloat("avionics_vcc", result[3] / 1000.0)
    power_node.setFloat("main_vcc", result[4] / 1000.0)
    power_node.setFloat("cell_vcc", result[5] / 1000.0)
    power_node.setFloat("main_amps", result[6] / 1000.0)
    power_node.setInt("total_mah", result[7] * 10.0)

    return index

def unpack_system_health_v5(buf):
    result = struct.unpack(system_health_v5_fmt, buf)

    index = result[0]
    
    status_node.setFloat("frame_time", result[1])
    status_node.setFloat("system_load_avg", result[2] / 100.0)
    power_node.setFloat("avionics_vcc", result[3] / 1000.0)
    power_node.setFloat("main_vcc", result[4] / 1000.0)
    power_node.setFloat("cell_vcc", result[5] / 1000.0)
    power_node.setFloat("main_amps", result[6] / 1000.0)
    power_node.setInt("total_mah", result[7] * 10.0)

    return index

def pack_payload_bin(index):
    buf = struct.pack(payload_v3_fmt,
                      index,
                      status_node.getFloat('frame_time'),
                      payload_node.getFloat("trigger_num"))
    return wrap_packet(PAYLOAD_PACKET_V3, buf)

def pack_payload_dict(index):
    row = dict()
    row['timestamp'] = payload_node.getFloat('timestamp')
    row['trigger_num'] = payload_node.getInt('trigger_num')
    return row

def pack_payload_csv(index):
    row = dict()
    row['timestamp'] = '%.4f' % payload_node.getFloat('timestamp')
    row['trigger_num'] = '%d' % payload_node.getInt('trigger_num')
    keys = ['timestamp', 'trigger_num']
    return row, keys

def unpack_payload_v2(buf):
    result = struct.unpack(payload_v2_fmt, buf)

    index = result[0]
    payload_node.setFloat("timestamp", result[1])
    payload_node.setInt("trigger_num", result[2])

    return index

def unpack_payload_v3(buf):
    result = struct.unpack(payload_v3_fmt, buf)

    index = result[0]
    payload_node.setFloat("timestamp", result[1])
    payload_node.setInt("trigger_num", result[2])

    return index

def pack_event_bin(message):
    global imu_timestamp

    # support an index value, but for now it will always be zero
    event_v1_fmt = '<BdB%ds' % len(message)
    buf = struct.pack(event_v1_fmt, 0, imu_timestamp, len(message),
                      str.encode(message))
    return wrap_packet(EVENT_PACKET_V1, buf)

def pack_event_dict(index):
    row = dict()
    row['timestamp'] = event_node.getFloat('timestamp')
    row['message'] = event_node.getString('message')
    return row
    
def pack_event_csv(index):
    row = dict()
    row['timestamp'] = '%.4f' % event_node.getFloat('timestamp')
    row['message'] = event_node.getString('message')
    keys = ['timestamp', 'message']
    return row, keys
    
def unpack_event_v1(buf):
    #print 'buf len:', len(buf)
    # unpack without knowing full size
    (index, timestamp, size) = struct.unpack("<BdB", buf[:10])
    #print 'unpack header len:', struct.calcsize("<BdB")
    #print 'expected size:', size
    #print 'maybe the message:', buf[10:]
    message = struct.unpack("%ds" % size, buf[10:])
    # print('message:', timestamp, message[0])
    
    #result = struct.unpack(event_v1_fmt, buf)
    #index = result[0]
    m = re.match('get: (.*)$', message[0].decode())
    if m:
        (prop, value) = m.group(1).split(',')
        # print prop, value
        # absolute path
        parts = prop.split('/')
        node_path = '/'.join(parts[0:-1])
        if node_path == '':
            node_path = '/'
        node = getNode(node_path, True)
        name = parts[-1]
        node.setString(name, value)
    event_node.setFloat("timestamp", timestamp)
    event_node.setString("message", message[0].decode())

    #print 'end of unpack event'
    return index

def pack_command_bin(sequence, message):
    if len(message) > 255:
        print("Error: command message too long, len =", len(message))
        message = 'command too long'
    # support an index value, but for now it will always be zero
    command_v1_fmt = '<BB%ds' % len(message)
    buf = struct.pack(command_v1_fmt, sequence & 0xFF, len(message), message)
    return wrap_packet(COMMAND_PACKET_V1, buf)

def unpack_command_v1(buf):
    # unpack without knowing full size
    (sequence, size) = struct.unpack("<BB", buf[:2])
    message = struct.unpack("%ds" % size, buf[2:])
    return sequence, message[0]


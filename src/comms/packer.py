import re
import struct

from props import getNode

from packet_id import *

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
gps_v1_fmt = '<dddfhhhdBB'
gps_v2_fmt = '<BdddfhhhdBB'
gps_v3_fmt = '<BdddfhhhdBHHHB'

imu_nodes = []
imu_v1_fmt = '<dfffffffffB'
imu_v2_fmt = '<dfffffffffhB'
imu_v3_fmt = '<BdfffffffffhB'

airdata_nodes = []
pos_node = getNode("/position", True)
pos_pressure_node = getNode("/position/pressure", True)
pos_combined_node = getNode("/position/combined", True)
vel_node = getNode("/velocity", True)
wind_node = getNode("/filters/wind", True)
airdata_v3_fmt = "<dHhhfhhHBBB"
airdata_v4_fmt = "<dHhhffhhHBBB"
airdata_v5_fmt = "<BdHhhffhHBBB"

filter_nodes = []
remote_link_node = getNode("/comms/remote_link", True)
filter_v1_fmt = "<dddfhhhhhhBB"
filter_v2_fmt = "<BdddfhhhhhhBB"
filter_v3_fmt = "<BdddfhhhhhhhhhhhhBB"

NUM_ACTUATORS = 8
act_node = getNode("/actuators", True)
act_v1_fmt = "<dhhHhhhhhB"
act_v2_fmt = "<BdhhHhhhhhB"

pilot_nodes = []
pilot_v1_fmt = "<dhhHhhhhhB"
pilot_v2_fmt = "<BdhhhhhhhhB"

status_node = getNode("/status", True)
ap_node = getNode("/autopilot", True)
targets_node = getNode("/autopilot/targets", True)
tecs_node = getNode("/autopilot/tecs", True)
task_node = getNode("/task", True)
route_node = getNode("/task/route", True)
active_node = getNode("/task/route/active", True)
home_node = getNode("/task/home", True)
circle_node = getNode("/task/circle", True)
ap_status_v1_fmt = "<dhhHhhhhddHHB"
ap_status_v2_fmt = "<dhhHhhhhHddHHB"
ap_status_v3_fmt = "<BdhhHhhhhHHddHHB"
ap_status_v4_fmt = "<BdhhHHhhHHddHHB"
ap_status_v5_fmt = "<BdBhhHhhhHHddHHB"

apm2_node = getNode("/sensors/APM2", True)
system_health_v2_fmt = "<dHHHHH"
system_health_v3_fmt = "<dHHHHHH"
system_health_v4_fmt = "<BdHHHHHH"

payload_node = getNode("/payload", True)
payload_v1_fmt = "<dH"
payload_v2_fmt = "<BdH"

raven_v1_fmt = "<BdHHHHHHHHHHffffB"

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
    # print "c0 =", c0, "c1 =", c1
    c0 = (c0 + size) & 0xff
    c1 = (c1 + c0) & 0xff
    # print "c0 =", c0, "c1 =", c1
    for i in range(0, size):
        c0 = (c0 + ord(buf[i])) & 0xff
        c1 = (c1 + c0) & 0xff
        # print "c0 =", c0, "c1 =", c1, '[', ord(buf[i]), ']'
    # print "c0 =", c0, "(", cksum0, ")", "c1 =", c1, "(", cksum1, ")"
    return (c0, c1)

# wrap payload in header bytes, id, length, payload, and compute checksums
def wrap_packet( packet_id, payload ):
    size = len(payload)
    buf = ''
    buf += chr(START_OF_MSG0)   # start of message sync bytes
    buf += chr(START_OF_MSG1)   # start of message sync bytes
    buf += chr(packet_id)       # packet id (1 byte)
    buf += chr(size)     # packet size (1 byte)
    buf += payload              # copy payload
    (cksum0, cksum1) = compute_cksum( packet_id, payload, size)
    buf += chr(cksum0)          # check sum byte 1
    buf += chr(cksum1)          # check sum byte 2
    return buf
    
def pack_gps_v3(index):
    if index >= len(gps_nodes):
        for i in range(len(gps_nodes),index+1):
            path = '/sensors/gps[%d]' % i
            gps_nodes.append( getNode(path, True) )
    node = gps_nodes[index]
    buf = struct.pack(gps_v3_fmt,
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
                      node.getFloat('horiz_accuracy_m') * 100,
                      node.getFloat('vert_accuracy_m') * 100,
                      node.getFloat('pdop') * 100,
                      node.getInt('fixType'))
    return wrap_packet(GPS_PACKET_V3, buf)

def pack_gps_text(index, delim=','):
    gps_node = getNode('/sensors/gps[%d]' % index, True)
    data = [ '%.4f' % gps_node.getFloat('timestamp'),
	     '%.10f' % gps_node.getFloat('latitude_deg'),
             '%.10f' % gps_node.getFloat('longitude_deg'),
             '%.2f' % gps_node.getFloat('altitude_m'),
	     '%.4f' % gps_node.getFloat('vn_ms'),
             '%.4f' % gps_node.getFloat('ve_ms'),
             '%.4f' % gps_node.getFloat('vd_ms'),
	     '%.3f' % gps_node.getFloat('unix_time_sec'),
             '%d' % gps_node.getInt('satellites'),
             '%.2f' % gps_node.getFloat('horiz_accuracy_m'),
             '%.2f' % gps_node.getFloat('vert_accuracy_m'),
             '%.2f' % gps_node.getFloat('pdop'),
             '%d' % gps_node.getInt('fixType') ]
    return delim.join(data)
    
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
    
def unpack_gps_v1(buf):
    index = 0
    if index >= len(gps_nodes):
        for i in range(len(gps_nodes),index+1):
            path = '/sensors/gps[%d]' % i
            gps_nodes.append( getNode(path, True) )
    node = gps_nodes[index]
    
    result = struct.unpack(gps_v1_fmt, buf)
    
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

    return index

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

def pack_imu_v3(index):
    global imu_timestamp
    
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
                      int(round(node.getFloat("temp_C") * 10.0)),
                      0)
    return wrap_packet(IMU_PACKET_V3, buf)

def pack_imu_text(index, delim=','):
    imu_node = getNode('/sensors/imu[%d]' % index, True)
    data = [ '%.4f' % imu_node.getFloat('timestamp'),
	     '%.4f' % imu_node.getFloat('p_rad_sec'),
	     '%.4f' % imu_node.getFloat('q_rad_sec'),
	     '%.4f' % imu_node.getFloat('r_rad_sec'),
	     '%.4f' % imu_node.getFloat('ax_mps_sec'),
	     '%.4f' % imu_node.getFloat('ay_mps_sec'),
	     '%.4f' % imu_node.getFloat('az_mps_sec'),
	     '%.3f' % imu_node.getFloat('hx'),
             '%.3f' % imu_node.getFloat('hy'),
             '%.3f' % imu_node.getFloat('hz'),
	     '%.1f' % imu_node.getFloat('temp_C'),
             '%d' % imu_node.getInt('status') ]
    return delim.join(data)
    
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
    
def unpack_imu_v1(buf):
    index = 0
    if index >= len(imu_nodes):
        for i in range(len(imu_nodes),index+1):
            path = '/sensors/imu[%d]' % i
            imu_nodes.append( getNode(path, True) )
    node = imu_nodes[index]

    result = struct.unpack(imu_v1_fmt, buf)
    
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

    return index

def unpack_imu_v2(buf):
    index = 0
    if index >= len(imu_nodes):
        for i in range(len(imu_nodes),index+1):
            path = '/sensors/imu[%d]' % i
            imu_nodes.append( getNode(path, True) )
    node = imu_nodes[index]

    result = struct.unpack(imu_v2_fmt, buf)

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

    return index

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
    return wrap_packet(AIRDATA_PACKET_V5, buf)

def pack_airdata_text(index, delim=','):
    airdata_node = getNode('/sensors/airdata[%d]' % index, True)
    data = [ '%.4f' % airdata_node.getFloat('timestamp'),
	     '%.1f' % airdata_node.getFloat('pressure_mbar'),
             '%.1f' % airdata_node.getFloat('temp_degC'),
	     '%.1f' % vel_node.getFloat('airspeed_smoothed_kt'),
	     '%.2f' % pos_pressure_node.getFloat('altitude_smoothed_m'),
             '%.2f' % pos_combined_node.getFloat('altitude_true_m'),
	     '%.2f' % (vel_node.getFloat('pressure_vertical_speed_fps')*60.0),
	     '%.1f' % wind_node.getFloat('wind_dir_deg'),
	     '%.1f' % wind_node.getFloat('wind_speed_kt'),
             '%.2f' % wind_node.getFloat('pitot_scale_factor'),
             '%d' % airdata_node.getInt('status') ]
    return delim.join(data)

def pack_airdata_csv(index):
    airdata_node = getNode('/sensors/airdata[%d]' % index, True)
    row = dict()
    row['timestamp'] = '%.4f' % airdata_node.getFloat('timestamp')
    row['pressure_mbar'] = '%.1f' % airdata_node.getFloat('pressure_mbar')
    row['temp_C'] = '%.1f' % airdata_node.getFloat('temp_degC')
    row['airspeed_smoothed_kt'] = '%.1f' % vel_node.getFloat('airspeed_smoothed_kt')
    row['altitude_smoothed_m'] = '%.2f' % pos_pressure_node.getFloat('altitude_smoothed_m')
    row['altitude_true_m'] = '%.2f' % pos_combined_node.getFloat('altitude_true_m')
    row['wind_dir_deg'] = '%.1f' % wind_node.getFloat('wind_dir_deg')
    row['wind_speed_kt'] = '%.1f' % wind_node.getFloat('wind_speed_kt')
    row['pitot_scale_factor'] = '%.2f' % wind_node.getFloat('pitot_scale_factor')
    row['status'] = '%d' % airdata_node.getInt('status')
    keys = ['timestamp', 'pressure_mbar', 'temp_C', 'airspeed_smoothed_kt',
            'altitude_smoothed_m', 'altitude_true_m',
            'wind_dir_deg', 'wind_speed_kt', 'pitot_scale_factor', 'status']
    return row, keys

def unpack_airdata_v3(buf):
    index = 0
    if index >= len(airdata_nodes):
        for i in range(len(airdata_nodes),index+1):
            path = '/sensors/airdata[%d]' % i
            airdata_nodes.append( getNode(path, True) )
    node = airdata_nodes[index]

    result = struct.unpack(airdata_v3_fmt, buf)
    
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

    return index

def unpack_airdata_v4(buf):
    index = 0
    if index >= len(airdata_nodes):
        for i in range(len(airdata_nodes),index+1):
            path = '/sensors/airdata[%d]' % i
            airdata_nodes.append( getNode(path, True) )
    node = airdata_nodes[index]

    result = struct.unpack(airdata_v4_fmt, buf)
    
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

    return index

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
    node.setFloat("temp_degC", result[3] / 100.0)
    vel_node.setFloat("airspeed_smoothed_kt", result[4] / 100.0)
    pos_pressure_node.setFloat("altitude_smoothed_m", result[5])
    pos_combined_node.setFloat("altitude_true_m", result[6])
    vel_node.setFloat("pressure_vertical_speed_fps", (result[7] / 10.0) / 60.0)
    wind_node.setFloat("wind_dir_deg", result[8] / 100.0)
    wind_node.setFloat("wind_speed_kt", result[9] / 4.0)
    wind_node.setFloat("pitot_scale_factor", result[10] / 100.0)
    node.setInt("status", result[11])

    return index

def pack_filter_v3(index):
    if index >= len(filter_nodes):
        for i in range(len(filter_nodes),index+1):
            path = '/filters/filter[%d]' % i
            filter_nodes.append( getNode(path, True) )
    node = filter_nodes[index]

    buf = struct.pack(filter_v3_fmt,
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
    return wrap_packet(FILTER_PACKET_V3, buf)

def pack_filter_text(index, delim=','):
    filter_node = getNode('/filters/filter[%d]' % index, True)
    data = [ '%.4f' % filter_node.getFloat('timestamp'),
	     '%.10f' % filter_node.getFloat('latitude_deg'),
             '%.10f' % filter_node.getFloat('longitude_deg'),
             '%.2f' % filter_node.getFloat('altitude_m'),
	     '%.4f' % filter_node.getFloat('vn_ms'),
             '%.4f' % filter_node.getFloat('ve_ms'),
             '%.4f' % filter_node.getFloat('vd_ms'),
	     '%.2f' % filter_node.getFloat('roll_deg'),
             '%.2f' % filter_node.getFloat('pitch_deg'),
             '%.2f' % filter_node.getFloat('heading_deg'),
             '%.4f' % filter_node.getFloat('p_bias'),
             '%.4f' % filter_node.getFloat('q_bias'),
             '%.4f' % filter_node.getFloat('r_bias'),
             '%.3f' % filter_node.getFloat('ax_bias'),
             '%.3f' % filter_node.getFloat('ay_bias'),
             '%.3f' % filter_node.getFloat('az_bias'),
	     '%d' % filter_node.getInt('status') ]
    return delim.join(data)

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
    row['roll_deg'] = '%.2f' % filter_node.getFloat('roll_deg')
    row['pitch_deg'] = '%.2f' % filter_node.getFloat('pitch_deg')
    row['heading_deg'] = '%.2f' % filter_node.getFloat('heading_deg')
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

def unpack_filter_v1(buf):
    index = 0
    if index >= len(filter_nodes):
        for i in range(len(filter_nodes),index+1):
            path = '/filters/filter[%d]' % i
            filter_nodes.append( getNode(path, True) )
    node = filter_nodes[index]

    result = struct.unpack(filter_v1_fmt, buf)
    
    node.setFloat("timestamp", result[0])
    node.setFloat("latitude_deg", result[1])
    node.setFloat("longitude_deg", result[2])
    node.setFloat("altitude_m", result[3])
    node.setFloat("vn_ms", result[4] / 100.0)
    node.setFloat("ve_ms", result[5] / 100.0)
    node.setFloat("vd_ms", result[6] / 100.0)
    node.setFloat("roll_deg", result[7] / 10.0)
    node.setFloat("pitch_deg", result[8] / 10.0)
    node.setFloat("heading_deg", result[9] / 10.0)
    remote_link_node.setInt("sequence_num", result[10])
    node.setInt("status", result[11])

    return index

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

def pack_act_v2(index):
    if index > 0:
        return
    buf = struct.pack(act_v2_fmt,
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
    return wrap_packet(ACTUATOR_PACKET_V2, buf)

def pack_act_text(index, delim=','):
    data = [ '%.4f' % act_node.getFloat('timestamp'),
	     '%.3f' % act_node.getFloat('aileron'),
	     '%.3f' % act_node.getFloat('elevator'),
	     '%.3f' % act_node.getFloat('throttle'),
	     '%.3f' % act_node.getFloat('rudder'),
	     '%.3f' % act_node.getFloat('channel5'),
	     '%.3f' % act_node.getFloat('flaps'),
	     '%.3f' % act_node.getFloat('channel7'),
	     '%.3f' % act_node.getFloat('channel8'),
	     '%d' % act_node.getInt('status') ]
    return delim.join(data)

def pack_act_csv(index):
    row = dict()
    row['timestamp'] = '%.4f' % act_node.getFloat('timestamp')
    row['aileron_norm'] = '%.3f' % act_node.getFloat('aileron')
    row['elevator_norm'] = '%.3f' % act_node.getFloat('elevator')
    row['throttle_norm'] = '%.3f' % act_node.getFloat('throttle')
    row['rudder_norm'] = '%.3f' % act_node.getFloat('rudder')
    row['channel5_norm'] = '%.3f' % act_node.getFloat('channel5')
    row['flaps_norm'] = '%.3f' % act_node.getFloat('flaps')
    row['channel7_norm'] = '%.3f' % act_node.getFloat('channel7')
    row['channel8_norm'] = '%.3f' % act_node.getFloat('channel8')
    row['status'] = '%d' % act_node.getInt('status')
    keys = ['timestamp', 'aileron_norm', 'elevator_norm', 'throttle_norm',
            'rudder_norm', 'channel5_norm', 'flaps_norm', 'channel7_norm',
            'channel8_norm', 'status']
    return row, keys

def unpack_act_v1(buf):
    result = struct.unpack(act_v1_fmt, buf)
    act_node.setFloat("timestamp", result[0])
    act_node.setFloat("aileron", result[1] / 30000.0)
    act_node.setFloat("elevator", result[2] / 30000.0)
    act_node.setFloat("throttle", result[3] / 60000.0)
    act_node.setFloat("rudder", result[4] / 30000.0)
    act_node.setFloat("channel5", result[5] / 30000.0)
    act_node.setFloat("flaps", result[6] / 30000.0)
    act_node.setFloat("channel7", result[7] / 30000.0)
    act_node.setFloat("channel8", result[8] / 30000.0)
    act_node.setInt("status", result[9])
    return 0

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
                      int(node.getFloatEnum("channel", 0) * 20000),
                      int(node.getFloatEnum("channel", 1) * 20000),
                      int(node.getFloatEnum("channel", 2) * 20000),
                      int(node.getFloatEnum("channel", 3) * 20000),
                      int(node.getFloatEnum("channel", 4) * 20000),
                      int(node.getFloatEnum("channel", 5) * 20000),
                      int(node.getFloatEnum("channel", 6) * 20000),
                      int(node.getFloatEnum("channel", 7) * 20000),
                      0)
    return wrap_packet(PILOT_INPUT_PACKET_V2, buf)

def pack_pilot_text(index, delim=','):
    pilot_node = getNode('/sensors/pilot_input[%d]' % index, True)
    data = [ '%.4f' % pilot_node.getFloat('timestamp'),
	     '%.3f' % pilot_node.getFloatEnum('channel', 0),
	     '%.3f' % pilot_node.getFloatEnum('channel', 1),
	     '%.3f' % pilot_node.getFloatEnum('channel', 2),
	     '%.3f' % pilot_node.getFloatEnum('channel', 3),
	     '%.3f' % pilot_node.getFloatEnum('channel', 4),
	     '%.3f' % pilot_node.getFloatEnum('channel', 5),
	     '%.3f' % pilot_node.getFloatEnum('channel', 6),
	     '%.3f' % pilot_node.getFloatEnum('channel', 7),
	     '%d' % pilot_node.getInt('status') ]
    return delim.join(data)

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

    node.setFloat("timestamp", result[0])
    node.setFloat("aileron", result[1] / 30000.0)
    node.setFloat("elevator", result[2] / 30000.0)
    node.setFloat("throttle", result[3] / 60000.0)
    node.setFloat("rudder", result[4] / 30000.0)
    node.setBool("manual", result[5] / 30000.0 > 0.5)
    node.setFloatEnum("channel", 5, result[6] / 30000.0)
    node.setFloatEnum("channel", 6, result[7] / 30000.0)
    node.setFloatEnum("channel", 7, result[8] / 30000.0)
    node.setInt("status", result[9])

    return index

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

def pack_ap_status_v5(index):
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
    elif counter == route_size + 1:
        wp_lon = home_node.getFloat("longitude_deg")
        wp_lat = home_node.getFloat("latitude_deg")
        wp_index = 65535

    #print index,                   status_node.getFloat('frame_time'),                      int(targets_node.getFloat("groundtrack_deg") * 10),                      int(targets_node.getFloat("roll_deg") * 10),                      int(target_msl_ft),                      int(targets_node.getFloat("climb_rate_fps") * 10),                      int(targets_node.getFloat("pitch_deg") * 10),                      int(targets_node.getFloat("the_dot") * 1000),                      int(targets_node.getFloat("airspeed_kt") * 10),                      int(task_node.getFloat("flight_timer")),                      route_node.getInt("target_waypoint_idx"),                      wp_lon,                      wp_lat,                      wp_index,                      route_size,                      remote_link_node.getInt("sequence_num")
    buf = struct.pack(ap_status_v5_fmt,
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
                      remote_link_node.getInt("sequence_num"))
    # print 'B:', index, flags, remote_link_node.getInt("sequence_num")
    # print 'd:', status_node.getFloat('frame_time'), wp_lon, wp_lat
    # print 'h:', int(round(targets_node.getFloat("groundtrack_deg") * 10)), int(round(targets_node.getFloat("roll_deg") * 10)),
    # print 'H:', int(round(target_msl_ft)), int(round(ground_m)),
    # int(round(targets_node.getFloat("pitch_deg") * 10)), int(round(targets_node.getFloat("airspeed_kt") * 10)), int(round(task_node.getFloat("flight_timer"))), route_node.getInt("target_waypoint_idx"), wp_index, route_size, 
    return wrap_packet(AP_STATUS_PACKET_V5, buf)

def pack_ap_status_text(index, delim=','):
    data = [ '%.4f' % targets_node.getFloat('timestamp'),
             '%d' % ap_node.getBool("master_switch"),
             '%d' % ap_node.getBool("pilot_pass_through"),
	     '%.2f' % targets_node.getFloat('groundtrack_deg'),
             '%.2f' % targets_node.getFloat('roll_deg'),
	     '%.2f' % targets_node.getFloat('altitude_msl_ft'),
             '%.2f' % targets_node.getFloat('climb_rate_fps'),
	     '%.2f' % targets_node.getFloat('pitch_deg'),
             '%.2f' % targets_node.getFloat('theta_dot'),
	     '%.1f' % targets_node.getFloat('airspeed_kt') ]
    return delim.join(data)

def pack_ap_status_csv(index):
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

def unpack_ap_status_v1(buf):
    result = struct.unpack(ap_status_v1_fmt, buf)

    targets_node.setFloat("timestamp", result[0])
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

    return 0

def unpack_ap_status_v2(buf):
    result = struct.unpack(ap_status_v2_fmt, buf)

    targets_node.setFloat("timestamp", result[0])
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

    return 0

def unpack_ap_status_v3(buf):
    result = struct.unpack(ap_status_v3_fmt, buf)

    index = result[0]

    wp_lon = result[11]
    wp_lat = result[12]
    wp_index = result[13]
    route_size = result[14]
    
    targets_node.setFloat("timestamp", result[1])
    targets_node.setFloat("groundtrack_deg", result[2] / 10.0)
    targets_node.setFloat("roll_deg", result[3] / 10.0)
    targets_node.setFloat("altitude_msl_ft", result[4])
    targets_node.setFloat("climb_rate_fps", result[5] / 10.0)
    targets_node.setFloat("pitch_deg", result[6] / 10.0)
    targets_node.setFloat("theta_dot", result[7] / 1000.0)
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

def pack_system_health_v4(index):
    dekamah = int(apm2_node.getFloat("extern_current_mah") / 10)
    if dekamah > 65535: dekamah = 65535 # prevent overflowing the structure
    buf = struct.pack(system_health_v4_fmt,
                      index,
                      status_node.getFloat('frame_time'),
                      int(status_node.getFloat("system_load_avg") * 100),
                      int(apm2_node.getFloat("board_vcc") * 1000),
                      int(apm2_node.getFloat("extern_volts") * 1000),
                      int(apm2_node.getFloat("extern_cell_volts") * 1000),
                      int(apm2_node.getFloat("extern_amps") * 1000),
                      dekamah)
    return wrap_packet(SYSTEM_HEALTH_PACKET_V4, buf)

def pack_system_health_text(index, delim=','):
    data = [ '%.4f' % status_node.getFloat('frame_time'),
	     '%.2f' % status_node.getFloat('system_load_avg'),
             '%.2f' % apm2_node.getFloat('board_vcc'),
	     '%.2f' % apm2_node.getFloat('extern_volts'),
             '%.2f' % apm2_node.getFloat('extern_cell_volts'),
	     '%.2f' % apm2_node.getFloat('extern_amps'),
             '%.0f' % apm2_node.getFloat('extern_current_mah') ]
    return delim.join(data)

def pack_system_health_csv(index):
    row = dict()
    row['timestamp'] = '%.4f' % status_node.getFloat('frame_time')
    row['system_load_avg'] = '%.2f' % status_node.getFloat('system_load_avg')
    row['board_vcc'] = '%.2f' % apm2_node.getFloat('board_vcc')
    row['extern_volts'] = '%.2f' % apm2_node.getFloat('extern_volts')
    row['extern_cell_volts'] = '%.2f' % apm2_node.getFloat('extern_cell_volts')
    row['extern_amps'] = '%.2f' % apm2_node.getFloat('extern_amps')
    row['extern_current_mah'] = '%.0f' % apm2_node.getFloat('extern_current_mah')
    keys = ['timestamp', 'system_load_avg', 'board_vcc', 'extern_volts',
            'extern_cell_volts', 'extern_amps', 'extern_current_mah']
    return row, keys

def unpack_system_health_v2(buf):
    result = struct.unpack(system_health_v2_fmt, buf)

    # imu_node.setFloat("timestamp", result[0]) # fixme? where to write this value?
    status_node.setFloat("system_load_avg", result[1] / 100.0)
    apm2_node.setFloat("board_vcc", result[2] / 1000.0)
    apm2_node.setFloat("extern_volts", result[3] / 1000.0)
    apm2_node.setFloat("extern_amps", result[5] / 1000.0)
    apm2_node.setFloat("extern_current_mah", result[6])

    return 0

def unpack_system_health_v3(buf):
    result = struct.unpack(system_health_v3_fmt, buf)

    status_node.setFloat("frame_time", result[0])
    status_node.setFloat("system_load_avg", result[1] / 100.0)
    apm2_node.setFloat("board_vcc", result[2] / 1000.0)
    apm2_node.setFloat("extern_volts", result[3] / 1000.0)
    apm2_node.setFloat("extern_cell_volts", result[4] / 1000.0)
    apm2_node.setFloat("extern_amps", result[5] / 1000.0)
    apm2_node.setFloat("extern_current_mah", result[6])

    return 0

def unpack_system_health_v4(buf):
    result = struct.unpack(system_health_v4_fmt, buf)

    index = result[0]
    
    status_node.setFloat("frame_time", result[1])
    status_node.setFloat("system_load_avg", result[2] / 100.0)
    apm2_node.setFloat("board_vcc", result[3] / 1000.0)
    apm2_node.setFloat("extern_volts", result[4] / 1000.0)
    apm2_node.setFloat("extern_cell_volts", result[5] / 1000.0)
    apm2_node.setFloat("extern_amps", result[6] / 1000.0)
    apm2_node.setInt("extern_current_mah", result[7] * 10.0)

    return index

def pack_payload_v2(index):
    buf = struct.pack(payload_v2_fmt,
                      index,
                      status_node.getFloat('frame_time'),
                      payload_node.getFloat("trigger_num"))
    return wrap_packet(PAYLOAD_PACKET_V2, buf)

def pack_payload_text(index, delim=','):
    data = [ '%.4f' % payload_node.getFloat('timestamp'),
	     '%d' % payload_node.getInt('trigger_num') ]
    return delim.join(data)

def pack_payload_csv(index):
    row = dict()
    row['timestamp'] = '%.4f' % payload_node.getFloat('timestamp')
    row['trigger_num'] = '%d' % payload_node.getInt('trigger_num')
    keys = ['timestamp', 'trigger_num']
    return row, keys

def unpack_payload_v1(buf):
    result = struct.unpack(payload_v1_fmt, buf)

    payload_node.setFloat("timestamp", result[0])
    payload_node.setInt("trigger_num", result[1])

    return 0
    
def unpack_payload_v2(buf):
    result = struct.unpack(payload_v2_fmt, buf)

    index = result[0]
    payload_node.setFloat("timestamp", result[1])
    payload_node.setInt("trigger_num", result[2])

    return index

# raven is currently a special airdata node, but it is much more than
# that.
def pack_raven_v1(index):
    global imu_timestamp
    
    if index >= len(airdata_nodes):
        for i in range(len(airdata_nodes),index+1):
            path = '/sensors/airdata[%d]' % i
            airdata_nodes.append( getNode(path, True) )
    node = airdata_nodes[index]
    buf = struct.pack(raven_v1_fmt,
                      index,
                      imu_timestamp,
                      node.getIntEnum("pots", 0),
                      node.getIntEnum("pots", 1),
                      node.getIntEnum("pots", 2),
                      node.getIntEnum("pots", 3),
                      node.getIntEnum("pots", 4),
                      node.getIntEnum("pots", 5),
                      node.getIntEnum("pots", 6),
                      node.getIntEnum("pots", 7),
                      node.getIntEnum("pots", 8),
                      node.getIntEnum("pots", 9),
                      node.getFloat("diff_pa"),
                      node.getFloat("pressure_mbar"),
                      node.getFloat("rpm0"),
                      node.getFloat("rpm1"),
                      0)
    return wrap_packet(RAVEN_PACKET_V1, buf)

def pack_raven_text(index, delim=','):
    node = getNode('/sensors/airdata[%d]' % index, True)
    data = [ '%.4f' % node.getFloat('timestamp'),
             '%d' % node.getIntEnum('pots', 0),
             '%d' % node.getIntEnum('pots', 1),
             '%d' % node.getIntEnum('pots', 2),
             '%d' % node.getIntEnum('pots', 3),
             '%d' % node.getIntEnum('pots', 4),
             '%d' % node.getIntEnum('pots', 5),
             '%d' % node.getIntEnum('pots', 6),
             '%d' % node.getIntEnum('pots', 7),
             '%d' % node.getIntEnum('pots', 8),
             '%d' % node.getIntEnum('pots', 9),
             '%.4f' % node.getFloat('diff_pa'),
	     '%.4f' % node.getFloat('pressure_mbar'),
             '%.2f' % node.getFloat('rpm0'),
             '%.2f' % node.getFloat('rpm1'),
             '%d' % node.getInt('status') ]
    return delim.join(data)
    
def pack_raven_csv(index):
    node = getNode('/sensors/airdata[%d]' % index, True)
    row = dict()
    row['timestamp'] = '%.4f' % node.getFloat('timestamp')
    row['pots[0]'] = '%d' % node.getIntEnum('pots', 0)
    row['pots[1]'] = '%d' % node.getIntEnum('pots', 1)
    row['pots[2]'] = '%d' % node.getIntEnum('pots', 2)
    row['pots[3]'] = '%d' % node.getIntEnum('pots', 3)
    row['pots[4]'] = '%d' % node.getIntEnum('pots', 4)
    row['pots[5]'] = '%d' % node.getIntEnum('pots', 5)
    row['pots[6]'] = '%d' % node.getIntEnum('pots', 6)
    row['pots[7]'] = '%d' % node.getIntEnum('pots', 7)
    row['pots[8]'] = '%d' % node.getIntEnum('pots', 8)
    row['pots[9]'] = '%d' % node.getIntEnum('pots', 9)
    row['diff_pa'] = '%.4f' % node.getFloat('diff_pa')
    row['pressure_mbar'] = '%.4f' % node.getFloat('pressure_mbar')
    row['rpm0'] = '%.2f' % node.getFloat('rpm0')
    row['rpm1'] = '%.2f' % node.getFloat('rpm1')
    row['status'] = '%d' % node.getInt('status')
    keys = ['timestamp', 'pots[0]', 'pots[1]', 'pots[2]', 'pots[3]',
            'pots[4]', 'pots[5]', 'pots[6]', 'pots[7]', 'pots[8]', 'pots[9]',
            'diff_pa', 'pressure_mbar', 'rpm0', 'rpm1', 'status']
    return row, keys
    
def unpack_raven_v1(buf):
    result = struct.unpack(raven_v1_fmt, buf)

    index = result[0]
    if index >= len(airdata_nodes):
        for i in range(len(airdata_nodes),index+1):
            path = '/sensors/airdata[%d]' % i
            airdata_nodes.append( getNode(path, True) )
    node = airdata_nodes[index]

    node.setFloat("timestamp", result[1])
    node.setIntEnum("pots", 0, result[2])
    node.setIntEnum("pots", 1, result[3])
    node.setIntEnum("pots", 2, result[4])
    node.setIntEnum("pots", 3, result[5])
    node.setIntEnum("pots", 4, result[6])
    node.setIntEnum("pots", 5, result[7])
    node.setIntEnum("pots", 6, result[8])
    node.setIntEnum("pots", 7, result[9])
    node.setIntEnum("pots", 8, result[10])
    node.setIntEnum("pots", 9, result[11])
    node.setFloat("diff_pa", result[12])
    node.setFloat("pressure_mbar", result[13])
    node.setFloat("rpm0", result[14])
    node.setFloat("rpm1", result[15])
    node.setInt("status", result[16])

    return index

def pack_event_v1(message):
    global imu_timestamp

    # support an index value, but for now it will always be zero
    event_v1_fmt = '<BdB%ds' % len(message)
    buf = struct.pack(event_v1_fmt, 0, imu_timestamp, len(message), message)
    return wrap_packet(EVENT_PACKET_V1, buf)

def pack_event_text(index, delim=','):
    data = [ '%.4f' % event_node.getFloat('timestamp'),
             '%s' % event_node.getString('message') ]
    return delim.join(data)
    
def pack_event_csv(index):
    row = dict()
    row['timestamp'] = '%.4f' % event_node.getFloat('timestamp')
    row['message'] = '%s' % event_node.getString('message')
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
    #print 'message:', timestamp, message[0]
    
    #result = struct.unpack(event_v1_fmt, buf)
    #index = result[0]
    m = re.match('get: (.*)$', message[0])
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
    else:
        event_node.setFloat("timestamp", timestamp)
        event_node.setString("message", message[0])

    #print 'end of unpack event'
    return index

def pack_command_v1(sequence, message):
    if len(message) > 255:
        print "Error: command message too long, len =", len(message)
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


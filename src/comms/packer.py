import re
import struct

from props import getNode

from comms import aura_messages

# FIXME: we are hard coding status flag to zero in many places which
# means we aren't using them properly (and/or wasting bytes)

# FIXME: last_imu_time doesn't address possible multiple imu channels
# (same for other logging categories

ft2m = 0.3048
m2ft = 1.0 / ft2m

START_OF_MSG0 = 147
START_OF_MSG1 = 224
    
airdata_node = getNode("/sensors/airdata[0]", True)
filter_node = getNode("/filters/filter", True)
gps_node = getNode("/sensors/gps[0]", True)
imu_node = getNode("/sensors/imu[0]", True)
pilot_node = getNode("/sensors/pilot_input", True)
pos_node = getNode("/position", True)
pos_pressure_node = getNode("/position/pressure", True)
pos_combined_node = getNode("/position/combined", True)
power_node = getNode("/sensors/power", True)
vel_node = getNode("/velocity", True)
wind_node = getNode("/filters/wind", True)
remote_link_node = getNode("/comms/remote_link", True)

NUM_ACTUATORS = 8
act_node = getNode("/actuators", True)

status_node = getNode("/status", True)
ap_node = getNode("/autopilot", True)
targets_node = getNode("/autopilot/targets", True)
tecs_node = getNode("/autopilot/tecs", True)
task_node = getNode("/task", True)
route_node = getNode("/task/route", True)
active_node = getNode("/task/route/active", True)
home_node = getNode("/task/home", True)
circle_node = getNode("/task/circle/active", True)

power_node = getNode("/sensors/power", True)
payload_node = getNode("/payload", True)
event_node = getNode("/status/event", True)

# simple 2-byte checksum
def compute_cksum(self, id, buf, size):
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
def wrap_packet( self, packet_id, payload ):
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

class Packer():
    ap = aura_messages.ap_status_v7()
    act = aura_messages.actuator_v3()
    airdata = aura_messages.airdata_v7()
    filter = aura_messages.filter_v5()
    gps = aura_messages.gps_v4()
    health = aura_messages.system_health_v6()
    imu = aura_messages.imu_v5()
    pilot = aura_messages.pilot_v3()
    ap_buf = None
    act_buf = None
    airdata_buf = None
    filter_buf = None
    gps_buf = None
    health_buf = None
    imu_buf = None
    pilot_buf = None
    last_ap_time = -1.0
    last_act_time = -1.0
    last_airdata_time = -1.0
    last_filter_time = -1.0
    last_gps_time = -1.0
    last_health_time = -1.0
    last_imu_time = -1.0
    last_pilot_time = -1.0
    
    def __init__(self):
        pass

    def pack_airdata_bin(self, use_cached=False):
        airdata_time = airdata_node.getFloat("timestamp")
        if not use_cached and airdata_time > self.last_airdata_time:
            self.last_airdata_time = airdata_time
            self.airdata.index = 0
            self.airdata.timestamp_sec = airdata_time
            self.airdata.pressure_mbar = airdata_node.getFloat("pressure_mbar")
            self.airdata.temp_C = airdata_node.getFloat("temp_C")
            self.airdata.airspeed_smoothed_kt = vel_node.getFloat("airspeed_smoothed_kt")
            self.airdata.altitude_smoothed_m = pos_pressure_node.getFloat('altitude_smoothed_m')
            self.airdata.altitude_true_m = pos_combined_node.getFloat("altitude_true_m")
            self.airdata.pressure_vertical_speed_fps = vel_node.getFloat("pressure_vertical_speed_fps")
            self.airdata.wind_dir_deg = wind_node.getFloat("wind_dir_deg")
            self.airdata.wind_speed_kt = wind_node.getFloat("wind_speed_kt")
            self.airdata.pitot_scale_factor = wind_node.getFloat("pitot_scale_factor")
            self.airdata.error_count = airdata_node.getInt("error_count")
            self.airdata.status = airdata_node.getInt("status")
            self.airdata_buf = self.airdata.pack()
        return self.airdata_buf

    def pack_airdata_dict(self, index):
        airdata_node = getNode('/sensors/airdata[%d]' % index, True)
        row = dict()
        row['timestamp'] = airdata_node.getFloat('timestamp')
        row['pressure_mbar'] = airdata_node.getFloat('pressure_mbar')
        row['temp_C'] = airdata_node.getFloat('temp_C')
        row['airspeed_smoothed_kt'] = vel_node.getFloat('airspeed_smoothed_kt')
        row['altitude_smoothed_m'] = pos_pressure_node.getFloat('altitude_smoothed_m')
        row['altitude_true_m'] = pos_combined_node.getFloat('altitude_true_m')
        row["pressure_vertical_speed_fps"] = vel_node.getFloat("pressure_vertical_speed_fps")
        row['wind_dir_deg'] = wind_node.getFloat('wind_dir_deg')
        row['wind_speed_kt'] = wind_node.getFloat('wind_speed_kt')
        row['pitot_scale_factor'] = wind_node.getFloat('pitot_scale_factor')
        row['tecs_error_total'] = tecs_node.getFloat('error_total')
        row['tecs_error_diff'] = tecs_node.getFloat('error_diff')
        row['error_count'] = airdata_node.getFloat('error_count')
        # print('airdata error:', row['error_count'])
        row['status'] = airdata_node.getInt('status')
        return row

    def pack_airdata_csv(self, index):
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

    def unpack_airdata_v5(self, buf):
        air = aura_messages.airdata_v5(buf)

        if air.index > 0:
            printf("Warning: airdata index > 0 not supported")
        node = airdata_node

        node.setFloat("timestamp", air.timestamp_sec)
        node.setFloat("pressure_mbar", air.pressure_mbar)
        node.setFloat("temp_C", air.temp_C)
        vel_node.setFloat("airspeed_smoothed_kt", air.airspeed_smoothed_kt)
        pos_pressure_node.setFloat("altitude_smoothed_m", air.altitude_smoothed_m)
        pos_combined_node.setFloat("altitude_true_m", air.altitude_true_m)
        vel_node.setFloat("pressure_vertical_speed_fps", air.pressure_vertical_speed_fps)
        wind_node.setFloat("wind_dir_deg", air.wind_dir_deg)
        wind_node.setFloat("wind_speed_kt", air.wind_speed_kt)
        wind_node.setFloat("pitot_scale_factor", air.pitot_scale_factor)
        node.setInt("status", air.status)
        return air.index

    def unpack_airdata_v6(self, buf):
        air = aura_messages.airdata_v6(buf)

        if air.index > 0:
            printf("Warning: airdata index > 0 not supported")
        node = airdata_node

        node.setFloat("timestamp", air.timestamp_sec)
        node.setFloat("pressure_mbar", air.pressure_mbar)
        node.setFloat("temp_C", air.temp_C)
        vel_node.setFloat("airspeed_smoothed_kt", air.airspeed_smoothed_kt)
        pos_pressure_node.setFloat("altitude_smoothed_m", air.altitude_smoothed_m)
        pos_combined_node.setFloat("altitude_true_m", air.altitude_true_m)
        vel_node.setFloat("pressure_vertical_speed_fps", air.pressure_vertical_speed_fps)
        wind_node.setFloat("wind_dir_deg", air.wind_dir_deg)
        wind_node.setFloat("wind_speed_kt", air.wind_speed_kt)
        wind_node.setFloat("pitot_scale_factor", air.pitot_scale_factor)
        node.setInt("status", air.status)
        return air.index

    def unpack_airdata_v7(self, buf):
        air = aura_messages.airdata_v7(buf)

        if air.index > 0:
            printf("Warning: airdata index > 0 not supported")
        node = airdata_node

        node.setFloat("timestamp", air.timestamp_sec)
        node.setFloat("pressure_mbar", air.pressure_mbar)
        node.setFloat("temp_C", air.temp_C)
        vel_node.setFloat("airspeed_smoothed_kt", air.airspeed_smoothed_kt)
        pos_pressure_node.setFloat("altitude_smoothed_m", air.altitude_smoothed_m)
        pos_combined_node.setFloat("altitude_true_m", air.altitude_true_m)
        vel_node.setFloat("pressure_vertical_speed_fps", air.pressure_vertical_speed_fps)
        wind_node.setFloat("wind_dir_deg", air.wind_dir_deg)
        wind_node.setFloat("wind_speed_kt", air.wind_speed_kt)
        wind_node.setFloat("pitot_scale_factor", air.pitot_scale_factor)
        node.setInt("error_count", air.error_count)
        node.setInt("status", air.status)
        return air.index

    # FIXME: think about how we are dealing with skips and gps's lower rate?
    def pack_gps_bin(self, use_cached=False):
        gps_time = gps_node.getFloat("timestamp")
        if use_cached:
            return self.gps_buf
        elif (gps_time > self.last_gps_time) or self.gps_buf is None:
            self.last_gps_time = gps_time
            self.gps.index = 0
            self.gps.timestamp_sec = gps_time
            self.gps.latitude_deg = gps_node.getFloat("latitude_deg")
            self.gps.longitude_deg = gps_node.getFloat("longitude_deg")
            self.gps.altitude_m = gps_node.getFloat("altitude_m")
            self.gps.vn_ms = gps_node.getFloat("vn_ms")
            self.gps.ve_ms = gps_node.getFloat("ve_ms")
            self.gps.vd_ms = gps_node.getFloat("vd_ms")
            self.gps.unixtime_sec = gps_node.getFloat("unix_time_sec")
            self.gps.satellites = gps_node.getInt("satellites")
            hacc = gps_node.getFloat("horiz_accuracy_m")
            if hacc > 655: hacc = 655
            self.gps.horiz_accuracy_m = hacc
            vacc = gps_node.getFloat("vert_accuracy_m")
            if vacc > 655: vacc = 655
            self.gps.vert_accuracy_m = vacc
            self.gps.pdop = gps_node.getFloat("pdop")
            self.gps.fix_type = gps_node.getInt("FixType")
            self.gps_buf = self.gps.pack()
            return self.gps_buf
        else:
            return None

    def pack_gps_dict(self, index):
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

    def pack_gps_csv(self, index):
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

    def unpack_gps_v2(self, buf):
        gps = aura_messages.gps_v2(buf)

        if gps.index > 0:
            printf("Warning: gps index > 0 not supported")
        node = gps_node

        node.setFloat("timestamp", gps.timestamp_sec)
        node.setFloat("latitude_deg", gps.latitude_deg)
        node.setFloat("longitude_deg", gps.longitude_deg)
        node.setFloat("altitude_m", gps.altitude_m)
        node.setFloat("vn_ms", gps.vn_ms)
        node.setFloat("ve_ms", gps.ve_ms)
        node.setFloat("vd_ms", gps.vd_ms)
        node.setFloat("unix_time_sec", gps.unixtime_sec)
        node.setInt("satellites", gps.satellites)
        node.setInt("status", 0)
        return gps.index

    def unpack_gps_v3(self, buf):
        gps = aura_messages.gps_v3(buf)

        if gps.index > 0:
            printf("Warning: gps index > 0 not supported")
        node = gps_node

        node.setFloat("timestamp", gps.timestamp_sec)
        node.setFloat("latitude_deg", gps.latitude_deg)
        node.setFloat("longitude_deg", gps.longitude_deg)
        node.setFloat("altitude_m", gps.altitude_m)
        node.setFloat("vn_ms", gps.vn_ms)
        node.setFloat("ve_ms", gps.ve_ms)
        node.setFloat("vd_ms", gps.vd_ms)
        node.setFloat("unix_time_sec", gps.unixtime_sec)
        node.setInt("satellites", gps.satellites)
        node.setFloat('horiz_accuracy_m', gps.horiz_accuracy_m)
        node.setFloat('vert_accuracy_m', gps.vert_accuracy_m)
        node.setFloat('pdop', gps.pdop)
        node.setInt('fixType', gps.fix_type)
        node.setInt("status", 0)
        return gps.index

    def unpack_gps_v4(self, buf):
        gps = aura_messages.gps_v4(buf)

        if gps.index > 0:
            printf("Warning: gps index > 0 not supported")
        node = gps_node

        node.setFloat("timestamp", gps.timestamp_sec)
        node.setFloat("latitude_deg", gps.latitude_deg)
        node.setFloat("longitude_deg", gps.longitude_deg)
        node.setFloat("altitude_m", gps.altitude_m)
        node.setFloat("vn_ms", gps.vn_ms)
        node.setFloat("ve_ms", gps.ve_ms)
        node.setFloat("vd_ms", gps.vd_ms)
        node.setFloat("unix_time_sec", gps.unixtime_sec)
        node.setInt("satellites", gps.satellites)
        node.setFloat('horiz_accuracy_m', gps.horiz_accuracy_m)
        node.setFloat('vert_accuracy_m', gps.vert_accuracy_m)
        node.setFloat('pdop', gps.pdop)
        node.setInt('fixType', gps.fix_type)
        node.setInt("status", 0)
        return gps.index

    # only support primary imu for now
    def pack_imu_bin(self, use_cached=False):
        imu_time = imu_node.getFloat('timestamp')
        if not use_cached and imu_time > self.last_imu_time:
            self.last_imu_time = imu_time
            self.imu.index = 0
            self.imu.timestamp_sec = imu_time
            self.imu.p_rad_sec = imu_node.getFloat('p_rad_sec')
            self.imu.q_rad_sec = imu_node.getFloat('q_rad_sec')
            self.imu.r_rad_sec = imu_node.getFloat('r_rad_sec')
            self.imu.ax_mps_sec = imu_node.getFloat('ax_mps_sec')
            self.imu.ay_mps_sec = imu_node.getFloat('ay_mps_sec')
            self.imu.az_mps_sec = imu_node.getFloat('az_mps_sec')
            self.imu.hx = imu_node.getFloat('hx')
            self.imu.hy = imu_node.getFloat('hy')
            self.imu.hz = imu_node.getFloat('hz')
            self.imu.ax_raw = imu_node.getFloat('ax_raw')
            self.imu.ay_raw = imu_node.getFloat('ay_raw')
            self.imu.az_raw = imu_node.getFloat('az_raw')
            self.imu.hx_raw = imu_node.getFloat('hx_raw')
            self.imu.hy_raw = imu_node.getFloat('hy_raw')
            self.imu.hz_raw = imu_node.getFloat('hz_raw')
            self.imu.temp_C = imu_node.getFloat('temp_C')
            self.imu.status = imu_node.getInt('status')
            self.imu_buf = self.imu.pack()
        return self.imu_buf

    def pack_imu_dict(self, index):
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
        row['ax_raw'] = imu_node.getFloat('ax_raw')
        row['ay_raw'] = imu_node.getFloat('ay_raw')
        row['az_raw'] = imu_node.getFloat('az_raw')
        row['hx_raw'] = imu_node.getFloat('hx_raw')
        row['hy_raw'] = imu_node.getFloat('hy_raw')
        row['hz_raw'] = imu_node.getFloat('hz_raw')
        row['temp_C'] = imu_node.getFloat('temp_C')
        row['status'] = imu_node.getInt('status')
        return row

    def pack_imu_csv(self, index):
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
        row['ax_raw'] = '%.4f' % imu_node.getFloat('ax_raw')
        row['ay_raw'] = '%.4f' % imu_node.getFloat('ay_raw')
        row['az_raw'] = '%.4f' % imu_node.getFloat('az_raw')
        row['hx_raw'] = '%.3f' % imu_node.getFloat('hx_raw')
        row['hy_raw'] = '%.3f' % imu_node.getFloat('hy_raw')
        row['hz_raw'] = '%.3f' % imu_node.getFloat('hz_raw')
        row['temp_C'] = '%.1f' % imu_node.getFloat('temp_C')
        row['status'] = '%d' % imu_node.getInt('status')
        keys = ['timestamp', 'p_rad_sec', 'q_rad_sec', 'r_rad_sec',
                'ax_mps_sec', 'ay_mps_sec', 'az_mps_sec',
                'hx', 'hy', 'hz', 'ax_raw', 'ay_raw', 'az_raw',
                'hx_raw', 'hy_raw', 'hz_raw', 'temp_C', 'status']
        return row, keys

    def unpack_imu_v3(self, buf):
        imu = aura_messages.imu_v3(buf)

        if imu.index > 0:
            printf("Warning: imu index > 0 not supported")
        node = imu_node

        node.setFloat("timestamp", imu.timestamp_sec)
        node.setFloat("p_rad_sec", imu.p_rad_sec)
        node.setFloat("q_rad_sec", imu.q_rad_sec)
        node.setFloat("r_rad_sec", imu.r_rad_sec)
        node.setFloat("ax_mps_sec", imu.ax_mps_sec)
        node.setFloat("ay_mps_sec", imu.ay_mps_sec)
        node.setFloat("az_mps_sec", imu.az_mps_sec)
        node.setFloat("hx", imu.hx)
        node.setFloat("hy", imu.hy)
        node.setFloat("hz", imu.hz)
        node.setFloat("temp_C", imu.temp_C)
        node.setInt("status", imu.status)
        return imu.index

    def unpack_imu_v4(self, buf):
        imu = aura_messages.imu_v4(buf)

        if imu.index > 0:
            printf("Warning: imu index > 0 not supported")
        node = imu_node

        node.setFloat("timestamp", imu.timestamp_sec)
        node.setFloat("p_rad_sec", imu.p_rad_sec)
        node.setFloat("q_rad_sec", imu.q_rad_sec)
        node.setFloat("r_rad_sec", imu.r_rad_sec)
        node.setFloat("ax_mps_sec", imu.ax_mps_sec)
        node.setFloat("ay_mps_sec", imu.ay_mps_sec)
        node.setFloat("az_mps_sec", imu.az_mps_sec)
        node.setFloat("hx", imu.hx)
        node.setFloat("hy", imu.hy)
        node.setFloat("hz", imu.hz)
        node.setFloat("temp_C", imu.temp_C)
        node.setInt("status", imu.status)
        return imu.index

    def unpack_imu_v5(self, buf):
        imu = aura_messages.imu_v5(buf)

        if imu.index > 0:
            printf("Warning: imu index > 0 not supported")
        node = imu_node

        node.setFloat("timestamp", imu.timestamp_sec)
        node.setFloat("p_rad_sec", imu.p_rad_sec)
        node.setFloat("q_rad_sec", imu.q_rad_sec)
        node.setFloat("r_rad_sec", imu.r_rad_sec)
        node.setFloat("ax_mps_sec", imu.ax_mps_sec)
        node.setFloat("ay_mps_sec", imu.ay_mps_sec)
        node.setFloat("az_mps_sec", imu.az_mps_sec)
        node.setFloat("hx", imu.hx)
        node.setFloat("hy", imu.hy)
        node.setFloat("hz", imu.hz)
        node.setFloat("ax_raw", imu.ax_raw)
        node.setFloat("ay_raw", imu.ay_raw)
        node.setFloat("az_raw", imu.az_raw)
        node.setFloat("hx_raw", imu.hx_raw)
        node.setFloat("hy_raw", imu.hy_raw)
        node.setFloat("hz_raw", imu.hz_raw)
        node.setFloat("temp_C", imu.temp_C)
        node.setInt("status", imu.status)
        return imu.index

    def pack_filter_bin(self, use_cached=False):
        filter_time = filter_node.getFloat("timestamp")
        if (not use_cached and filter_time > self.last_filter_time) or self.filter_buf is None:
            self.last_filter_time = filter_time
            self.filter.index = 0
            self.filter.timestamp_sec = filter_time
            self.filter.latitude_deg = filter_node.getFloat("latitude_deg")
            self.filter.longitude_deg = filter_node.getFloat("longitude_deg")
            self.filter.altitude_m = filter_node.getFloat("altitude_m")
            self.filter.vn_ms = filter_node.getFloat("vn_ms")
            self.filter.ve_ms = filter_node.getFloat("ve_ms")
            self.filter.vd_ms = filter_node.getFloat("vd_ms")
            self.filter.roll_deg = filter_node.getFloat("roll_deg")
            self.filter.pitch_deg = filter_node.getFloat("pitch_deg")
            self.filter.yaw_deg = filter_node.getFloat("heading_deg")
            self.filter.p_bias = filter_node.getFloat("p_bias")
            self.filter.q_bias = filter_node.getFloat("q_bias")
            self.filter.r_bias = filter_node.getFloat("r_bias") 
            self.filter.ax_bias = filter_node.getFloat("ax_bias")
            self.filter.ay_bias = filter_node.getFloat("ay_bias")
            self.filter.az_bias = filter_node.getFloat("az_bias")
            self.filter.max_pos_cov = filter_node.getFloat("max_pos_cov")
            self.filter.max_vel_cov = filter_node.getFloat("max_vel_cov")
            self.filter.max_att_cov = filter_node.getFloat("max_att_cov")
            self.filter.sequence_num = remote_link_node.getInt("sequence_num")
            self.filter.status = filter_node.getInt("status")
            self.filter_buf = self.filter.pack()
        return self.filter_buf

    def pack_filter_dict(self, index):
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
        row['max_pos_cov'] = filter_node.getFloat('max_pos_cov')
        row['max_vel_cov'] = filter_node.getFloat('max_vel_cov')
        row['max_att_cov'] = filter_node.getFloat('max_att_cov')
        row['status'] = filter_node.getInt('status')
        return row

    def pack_filter_csv(self, index):
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

    def unpack_filter_v3(self, buf):
        nav = aura_messages.filter_v3(buf)

        if nav.index > 0:
            printf("Warning: nav index > 0 not supported")
        node = filter_node

        node.setFloat("timestamp", nav.timestamp_sec)
        node.setFloat("latitude_deg", nav.latitude_deg)
        node.setFloat("longitude_deg", nav.longitude_deg)
        node.setFloat("altitude_m", nav.altitude_m)
        node.setFloat("vn_ms", nav.vn_ms)
        node.setFloat("ve_ms", nav.ve_ms)
        node.setFloat("vd_ms", nav.vd_ms)
        node.setFloat("roll_deg", nav.roll_deg)
        node.setFloat("pitch_deg", nav.pitch_deg)
        node.setFloat("heading_deg", nav.yaw_deg)
        node.setFloat("p_bias", nav.p_bias)
        node.setFloat("q_bias", nav.q_bias)
        node.setFloat("r_bias", nav.r_bias)
        node.setFloat("ax_bias", nav.ax_bias)
        node.setFloat("ay_bias", nav.ay_bias)
        node.setFloat("az_bias", nav.az_bias)
        if nav.sequence_num >= 1:
            remote_link_node.setInt("sequence_num", nav.sequence_num)
        node.setInt("status", nav.status)

        return nav.index

    def unpack_filter_v4(self, buf):
        nav = aura_messages.filter_v4(buf)

        if nav.index > 0:
            printf("Warning: nav index > 0 not supported")
        node = filter_node

        node.setFloat("timestamp", nav.timestamp_sec)
        node.setFloat("latitude_deg", nav.latitude_deg)
        node.setFloat("longitude_deg", nav.longitude_deg)
        node.setFloat("altitude_m", nav.altitude_m)
        node.setFloat("vn_ms", nav.vn_ms)
        node.setFloat("ve_ms", nav.ve_ms)
        node.setFloat("vd_ms", nav.vd_ms)
        node.setFloat("roll_deg", nav.roll_deg)
        node.setFloat("pitch_deg", nav.pitch_deg)
        node.setFloat("heading_deg", nav.yaw_deg)
        node.setFloat("p_bias", nav.p_bias)
        node.setFloat("q_bias", nav.q_bias)
        node.setFloat("r_bias", nav.r_bias)
        node.setFloat("ax_bias", nav.ax_bias)
        node.setFloat("ay_bias", nav.ay_bias)
        node.setFloat("az_bias", nav.az_bias)
        if nav.sequence_num >= 1:
            remote_link_node.setInt("sequence_num", nav.sequence_num)
        node.setInt("status", nav.status)

        return nav.index

    def unpack_filter_v5(self, buf):
        nav = aura_messages.filter_v5(buf)

        if nav.index > 0:
            printf("Warning: nav index > 0 not supported")
        node = filter_node

        node.setFloat("timestamp", nav.timestamp_sec)
        node.setFloat("latitude_deg", nav.latitude_deg)
        node.setFloat("longitude_deg", nav.longitude_deg)
        node.setFloat("altitude_m", nav.altitude_m)
        node.setFloat("vn_ms", nav.vn_ms)
        node.setFloat("ve_ms", nav.ve_ms)
        node.setFloat("vd_ms", nav.vd_ms)
        node.setFloat("roll_deg", nav.roll_deg)
        node.setFloat("pitch_deg", nav.pitch_deg)
        node.setFloat("heading_deg", nav.yaw_deg)
        node.setFloat("p_bias", nav.p_bias)
        node.setFloat("q_bias", nav.q_bias)
        node.setFloat("r_bias", nav.r_bias)
        node.setFloat("ax_bias", nav.ax_bias)
        node.setFloat("ay_bias", nav.ay_bias)
        node.setFloat("az_bias", nav.az_bias)
        node.setFloat("max_pos_cov", nav.max_pos_cov)
        node.setFloat("max_vel_cov", nav.max_vel_cov)
        node.setFloat("max_att_cov", nav.max_att_cov)
        if nav.sequence_num >= 1:
            remote_link_node.setInt("sequence_num", nav.sequence_num)
        node.setInt("status", nav.status)

        return nav.index

    def pack_act_bin(self, use_cached=False):
        act_time = act_node.getFloat('timestamp')
        if not use_cached and act_time > self.last_act_time:
            self.last_act_time = act_time
            self.act.index = 0
            self.act.timestamp_sec = act_time
            self.act.aileron = act_node.getFloat("aileron")
            self.act.elevator = act_node.getFloat("elevator")
            self.act.throttle = act_node.getFloat("throttle")
            self.act.rudder = act_node.getFloat("rudder")
            self.act.channel5 = act_node.getFloat("channel5")
            self.act.flaps = act_node.getFloat("flaps")
            self.act.channel7 = act_node.getFloat("channel7")
            self.act.channel8 = act_node.getFloat("channel8")
            self.act.status = 0
            self.act_buf = self.act.pack()
        return self.act_buf

    def pack_act_dict(self, index):
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

    def pack_act_csv(self, index):
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

    def unpack_act_v2(self, buf):
        act = aura_messages.actuator_v2(buf)
        act_node.setFloat("timestamp", act.timestamp_sec)
        act_node.setFloat("aileron", act.aileron)
        act_node.setFloat("elevator", act.elevator)
        act_node.setFloat("throttle", act.throttle)
        act_node.setFloat("rudder", act.rudder)
        act_node.setFloat("channel5", act.channel5)
        act_node.setFloat("flaps", act.flaps)
        act_node.setFloat("channel7", act.channel7)
        act_node.setFloat("channel8", act.channel8)
        act_node.setInt("status", act.status)
        return act.index

    def unpack_act_v3(self, buf):
        act = aura_messages.actuator_v3(buf)
        act_node.setFloat("timestamp", act.timestamp_sec)
        act_node.setFloat("aileron", act.aileron)
        act_node.setFloat("elevator", act.elevator)
        act_node.setFloat("throttle", act.throttle)
        act_node.setFloat("rudder", act.rudder)
        act_node.setFloat("channel5", act.channel5)
        act_node.setFloat("flaps", act.flaps)
        act_node.setFloat("channel7", act.channel7)
        act_node.setFloat("channel8", act.channel8)
        act_node.setInt("status", act.status)
        return act.index

    def pack_pilot_bin(self, use_cached=False):
        pilot_time = pilot_node.getFloat('timestamp')
        if not use_cached and pilot_time > self.last_pilot_time:
            self.last_pilot_time = pilot_time
            self.pilot.index = 0
            self.pilot.timestamp_sec = pilot_time
            for i in range(8):
                self.pilot.channel[i] = pilot_node.getFloatEnum("channel", i)
            self.pilot.status = 0
            self.pilot_buf = self.pilot.pack()
        return self.pilot_buf

    def pack_pilot_dict(self, index):
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

    def pack_pilot_csv(self, index):
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

    def unpack_pilot_v2(self, buf):
        pilot = aura_messages.pilot_v2(buf)

        if pilot.index > 0:
            printf("Warning: pilot index > 0 not supported")
        node = pilot_node

        node.setFloat("timestamp", pilot.timestamp_sec)
        node.setFloatEnum("channel", 0, pilot.channel[0])
        node.setFloatEnum("channel", 1, pilot.channel[1])
        node.setFloatEnum("channel", 2, pilot.channel[2])
        node.setFloatEnum("channel", 3, pilot.channel[3])
        node.setFloatEnum("channel", 4, pilot.channel[4])
        node.setFloatEnum("channel", 5, pilot.channel[5])
        node.setFloatEnum("channel", 6, pilot.channel[6])
        node.setFloatEnum("channel", 7, pilot.channel[7])
        node.setInt("status", pilot.status)

        return pilot.index

    def unpack_pilot_v3(self, buf):
        pilot = aura_messages.pilot_v3(buf)

        if pilot.index > 0:
            printf("Warning: pilot index > 0 not supported")
        node = pilot_node

        node.setFloat("timestamp", pilot.timestamp_sec)
        node.setFloatEnum("channel", 0, pilot.channel[0])
        node.setFloatEnum("channel", 1, pilot.channel[1])
        node.setFloatEnum("channel", 2, pilot.channel[2])
        node.setFloatEnum("channel", 3, pilot.channel[3])
        node.setFloatEnum("channel", 4, pilot.channel[4])
        node.setFloatEnum("channel", 5, pilot.channel[5])
        node.setFloatEnum("channel", 6, pilot.channel[6])
        node.setFloatEnum("channel", 7, pilot.channel[7])
        node.setInt("status", pilot.status)

        return pilot.index

    def pack_ap_status_bin(self, use_cached=False):
        ap_time = status_node.getFloat("frame_time")
        if not use_cached and ap_time > self.last_ap_time:
            self.last_ap_time = ap_time
            self.ap.index = 0
            self.ap.timestamp_sec = ap_time
            # status flags (up to 8 could be supported)
            self.ap.flags = 0
            if ap_node.getBool("master_switch"):
                self.ap.flags += 1 # |= (1 << 0)
            if ap_node.getBool("pilot_pass_through"):
                self.ap.flags += 2 # |= (1 << 1)
            self.ap.groundtrack_deg = targets_node.getFloat("groundtrack_deg")
            self.ap.roll_deg = targets_node.getFloat("roll_deg")
            target_agl_ft = targets_node.getFloat("altitude_agl_ft")
            ground_m = pos_node.getFloat("altitude_ground_m")
            # if pressure based:
            #   error_m = pos_pressure_node.getFloat("pressure_error_m")
            #   target_msl_ft = (ground_m + error_m) * m2ft + target_agl_ft
            # else: ...
            self.ap.altitude_msl_ft = ground_m * m2ft + target_agl_ft
            self.ap.altitude_ground_m = ground_m
            self.ap.pitch_deg = targets_node.getFloat("pitch_deg")
            self.ap.airspeed_kt = targets_node.getFloat("airspeed_kt")
            self.ap.flight_timer = task_node.getFloat("flight_timer")
            self.ap.target_waypoint_idx = route_node.getInt("target_waypoint_idx")

            # Note: task_attribute is an overloaded (uint16_t) field!
            # There will be a better way figured out sometime in the
            # future.
            self.ap.task_attribute = 0
            
            # wp_counter will get incremented externally in the
            # remote_link message sender because each time we send a
            # serial message to the remote ground station is when we
            # want to advance to the next waypoint.
            counter = remote_link_node.getInt("wp_counter")
            self.ap.wp_longitude_deg = 0.0
            self.ap.wp_latitude_deg = 0.0
            self.ap.wp_index = 0
            self.ap.route_size = active_node.getInt("route_size")
            if self.ap.route_size > 0 and counter < self.ap.route_size:
                self.ap.wp_index = counter
                wp_path = "wpt[%d]" % self.ap.wp_index
                wp_node = active_node.getChild(wp_path, True)
                self.ap.wp_longitude_deg = wp_node.getFloat("longitude_deg")
                self.ap.wp_latitude_deg = wp_node.getFloat("latitude_deg")
            elif counter == self.ap.route_size:
                self.ap.wp_longitude_deg = circle_node.getFloat("longitude_deg")
                self.ap.wp_latitude_deg = circle_node.getFloat("latitude_deg")
                self.ap.wp_index = 65534
                self.ap.task_attribute = int(round(circle_node.getFloat("radius_m") * 10))
                if self.ap.task_attribute > 32767: self.ap.task_attribute = 32767
            elif counter == self.ap.route_size + 1:
                self.ap.wp_longitude_deg = home_node.getFloat("longitude_deg")
                self.ap.wp_latitude_deg = home_node.getFloat("latitude_deg")
                self.ap.wp_index = 65535

            # these id codes are ad-hoc and someday in the future
            # could be more formalized
            self.ap.task_id = 0 # code for unknown or not set
            if task_node.getString("current_task") == "circle":
                self.ap.task_id = 1
            elif task_node.getString("current_task") == "parametric":
                # draw like it's a circle
                self.ap.task_id = 1
            elif task_node.getString("current_task") == "route":
                self.ap.task_id = 2
            elif task_node.getString("current_task") == "land":
                self.ap.task_id = 3
            elif task_node.getString("current_task") == "calib_accels":
                self.ap.task_id = 4
                self.ap.task_attribute = task_node.getInt("calib_state")
            elif task_node.getString("current_task") == "calib_home":
                self.ap.task_id = 5
            elif task_node.getString("current_task") == "calib_mags":
                self.ap.task_id = 6
                self.ap.task_attribute = task_node.getInt("calib_state")
            self.ap.sequence_num = remote_link_node.getInt("sequence_num")
            self.ap_buf = self.ap.pack()
        return self.ap_buf

    wp_counter = 0
    def pack_ap_status_dict(self, index):
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
        row['flight_timer'] = task_node.getFloat("flight_timer")
        row['target_waypoint_idx'] = route_node.getInt("target_waypoint_idx")
        route_size = active_node.getInt("route_size")
        row['route_size'] = route_size
        row['task_attribute'] = 0.0
        if self.wp_counter < route_size:
            wp_node = active_node.getChild('wpt[%d]' % self.wp_counter, True)
            row['wpt_index'] = self.wp_counter
            row['wpt_longitude_deg'] = wp_node.getFloat("longitude_deg")
            row['wpt_latitude_deg'] = wp_node.getFloat("latitude_deg")
        elif self.wp_counter == route_size:
            row['wpt_index'] = 65534
            row['wpt_longitude_deg'] = circle_node.getFloat("longitude_deg")
            row['wpt_latitude_deg'] = circle_node.getFloat("latitude_deg")
            row['task_attribute'] = int(round(circle_node.getFloat("radius_m") * 10))
        elif self.wp_counter == route_size + 1:
            row['wpt_index'] = 65535
            row['wpt_longitude_deg'] = home_node.getFloat("longitude_deg")
            row['wpt_latitude_deg'] = home_node.getFloat("latitude_deg")
        row['current_task'] = task_node.getString("current_task")
        self.wp_counter += 1
        if self.wp_counter >= route_size + 2:
            self.wp_counter = 0
        return row

    def pack_ap_status_csv(self, index):
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

    def unpack_ap_status_v4(self, buf):
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
        if result[14] >= 1:
            remote_link_node.setInt("sequence_num", result[14])

        return index

    def unpack_ap_status_v5(self, buf):
        ap = aura_messages.ap_status_v5(buf)

        index = ap.index

        wp_lon = ap.wp_longitude_deg
        wp_lat = ap.wp_latitude_deg
        wp_index = ap.wp_index
        route_size = ap.route_size

        targets_node.setFloat("timestamp", ap.timestamp_sec)
        flags = ap.flags
        ap_node.setBool("master_switch", flags & (1<<0))
        ap_node.setBool("pilot_pass_through", flags & (1<<1))
        targets_node.setFloat("groundtrack_deg", ap.groundtrack_deg)
        targets_node.setFloat("roll_deg", ap.roll_deg)
        targets_node.setFloat("altitude_msl_ft", ap.altitude_msl_ft)
        pos_node.setFloat("altitude_ground_m", ap.altitude_ground_m)
        targets_node.setFloat("pitch_deg", ap.pitch_deg)
        targets_node.setFloat("airspeed_kt", ap.airspeed_kt)
        status_node.setFloat("flight_timer", ap.flight_timer)
        status_node.setBool("onboard_flight_timer", True)
        if route_size != active_node.getInt("route_size"):
            # route size change, zero all the waypoint coordinates
            for i in range(active_node.getInt("route_size")):
                wp_node = active_node.getChild('wpt[%d]' % i, True)
                wp_node.setFloat("longitude_deg", 0)
                wp_node.setFloat("latitude_deg", 0)
        route_node.setInt("target_waypoint_idx", ap.target_waypoint_idx)
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
        if ap.sequence_num >= 1:
            remote_link_node.setInt("sequence_num", ap.sequence_num)

        return index

    def unpack_ap_status_v6(self, buf):
        ap = aura_messages.ap_status_v6(buf)

        index = ap.index

        wp_lon = ap.wp_longitude_deg
        wp_lat = ap.wp_latitude_deg
        wp_index = ap.wp_index
        route_size = ap.route_size
        task_id = ap.task_id
        task_attrib = ap.task_attribute

        targets_node.setFloat("timestamp", ap.timestamp_sec)
        flags = ap.flags
        ap_node.setBool("master_switch", flags & (1<<0))
        ap_node.setBool("pilot_pass_through", flags & (1<<1))
        targets_node.setFloat("groundtrack_deg", ap.groundtrack_deg)
        targets_node.setFloat("roll_deg", ap.roll_deg)
        targets_node.setFloat("altitude_msl_ft", ap.altitude_msl_ft)
        pos_node.setFloat("altitude_ground_m", ap.altitude_ground_m)
        targets_node.setFloat("pitch_deg", ap.pitch_deg)
        targets_node.setFloat("airspeed_kt", ap.airspeed_kt)
        status_node.setFloat("flight_timer", ap.flight_timer)
        status_node.setBool("onboard_flight_timer", True)
        if route_size != active_node.getInt("route_size"):
            # route size change, zero all the waypoint coordinates
            for i in range(active_node.getInt("route_size")):
                wp_node = active_node.getChild('wpt[%d]' % i, True)
                wp_node.setFloat("longitude_deg", 0)
                wp_node.setFloat("latitude_deg", 0)
        route_node.setInt("target_waypoint_idx", ap.target_waypoint_idx)
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
            task_node.setString("current_task", "circle")
        elif task_id == 2:
            task_node.setString("current_task", "route")
        elif task_id == 3:
            task_node.setString("current_task", "land")
        else:
            task_node.setString("current_task", "unknown")

        active_node.setInt("route_size", route_size)
        if ap.sequence_num >= 1:
            remote_link_node.setInt("sequence_num", ap.sequence_num)

        return index

    def unpack_ap_status_v7(self, buf):
        ap = aura_messages.ap_status_v7(buf)

        index = ap.index

        wp_lon = ap.wp_longitude_deg
        wp_lat = ap.wp_latitude_deg
        wp_index = ap.wp_index
        route_size = ap.route_size
        task_id = ap.task_id
        task_attrib = ap.task_attribute

        targets_node.setFloat("timestamp", ap.timestamp_sec)
        flags = ap.flags
        ap_node.setBool("master_switch", flags & (1<<0))
        ap_node.setBool("pilot_pass_through", flags & (1<<1))
        targets_node.setFloat("groundtrack_deg", ap.groundtrack_deg)
        targets_node.setFloat("roll_deg", ap.roll_deg)
        targets_node.setFloat("altitude_msl_ft", ap.altitude_msl_ft)
        pos_node.setFloat("altitude_ground_m", ap.altitude_ground_m)
        targets_node.setFloat("pitch_deg", ap.pitch_deg)
        targets_node.setFloat("airspeed_kt", ap.airspeed_kt)
        status_node.setFloat("flight_timer", ap.flight_timer)
        status_node.setBool("onboard_flight_timer", True)
        if route_size != active_node.getInt("route_size"):
            # route size change, zero all the waypoint coordinates
            for i in range(active_node.getInt("route_size")):
                wp_node = active_node.getChild('wpt[%d]' % i, True)
                wp_node.setFloat("longitude_deg", 0)
                wp_node.setFloat("latitude_deg", 0)
        route_node.setInt("target_waypoint_idx", ap.target_waypoint_idx)
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
            task_node.setString("current_task", "circle")
        elif task_id == 2:
            task_node.setString("current_task", "route")
        elif task_id == 3:
            task_node.setString("current_task", "land")
        elif task_id == 4:
            task_node.setString("current_task", "calib_accels")
            task_node.setInt("calib_state", task_attrib)
            if task_attrib == 0:
                task_node.setString("calib_message", "place level, right side up")
            elif task_attrib == 1:
                task_node.setString("calib_message", "place upside down")
            elif task_attrib == 2:
                task_node.setString("calib_message", "place nose down")
            elif task_attrib == 3:
                task_node.setString("calib_message", "place nose up")
            elif task_attrib == 4:
                task_node.setString("calib_message", "place right wing down")
            elif task_attrib == 5:
                task_node.setString("calib_message", "place right wing up")
            elif task_attrib == 6:
                task_node.setString("calib_message", "computing calibration")
            elif task_attrib == 7:
                task_node.setString("calib_message", "accel calibration success")
            elif task_attrib == 8:
                task_node.setString("calib_message", "error, accel calibration failed")
            elif task_attrib == 99:
                task_node.setString("calib_message", "move 45 degrees off axis to arm")
        elif task_id == 5:
            task_node.setString("current_task", "calib_home")
        elif task_id == 6:
            task_node.setString("current_task", "calib_mags")
            task_node.setInt("calib_state", task_attrib)
        else:
            task_node.setString("current_task", "unknown")

        active_node.setInt("route_size", route_size)
        if ap.sequence_num >= 1:
            remote_link_node.setInt("sequence_num", ap.sequence_num)

        return index

    def pack_system_health_bin(self, use_cached=False):
        health_time = status_node.getFloat('frame_time')
        if not use_cached and health_time > self.last_health_time:
            self.last_health_time = health_time
            self.health.index = 0
            self.health.timestamp_sec = health_time
            self.health.system_load_avg = status_node.getFloat("system_load_avg")
            self.health.fmu_timer_misses = status_node.getInt("fmu_timer_misses")
            self.health.avionics_vcc = power_node.getFloat("avionics_vcc")
            self.health.main_vcc = power_node.getFloat("main_vcc")
            self.health.cell_vcc = power_node.getFloat("cell_vcc")
            self.health.main_amps = power_node.getFloat("main_amps")
            self.health.total_mah = power_node.getFloat("total_mah")
            self.health_buf = self.health.pack()
        return self.health_buf

    def pack_system_health_dict(self, index):
        row = dict()
        row['timestamp'] = status_node.getFloat('frame_time')
        row['system_load_avg'] = status_node.getFloat('system_load_avg')
        row['fmu_timer_misses'] = status_node.getFloat('fmu_timer_misses')
        row['avionics_vcc'] = power_node.getFloat('avionics_vcc')
        row['main_vcc'] = power_node.getFloat('main_vcc')
        row['cell_vcc'] = power_node.getFloat('cell_vcc')
        row['main_amps'] = power_node.getFloat('main_amps')
        row['total_mah'] = power_node.getFloat('total_mah')
        return row

    def pack_system_health_csv(self, index):
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

    def unpack_system_health_v4(self, buf):
        health = aura_messages.system_health_v4(buf)
        status_node.setFloat("frame_time", health.timestamp_sec)
        status_node.setFloat("system_load_avg", health.system_load_avg)
        power_node.setFloat("avionics_vcc", health.avionics_vcc)
        power_node.setFloat("main_vcc", health.main_vcc)
        power_node.setFloat("cell_vcc", health.cell_vcc)
        power_node.setFloat("main_amps", health.main_amps)
        power_node.setInt("total_mah", health.total_mah)
        return health.index

    def unpack_system_health_v5(self, buf):
        health = aura_messages.system_health_v5(buf)
        status_node.setFloat("frame_time", health.timestamp_sec)
        status_node.setFloat("system_load_avg", health.system_load_avg)
        power_node.setFloat("avionics_vcc", health.avionics_vcc)
        power_node.setFloat("main_vcc", health.main_vcc)
        power_node.setFloat("cell_vcc", health.cell_vcc)
        power_node.setFloat("main_amps", health.main_amps)
        power_node.setInt("total_mah", health.total_mah)
        return health.index

    def unpack_system_health_v6(self, buf):
        health = aura_messages.system_health_v6(buf)
        status_node.setFloat("frame_time", health.timestamp_sec)
        status_node.setFloat("system_load_avg", health.system_load_avg)
        status_node.setInt("fmu_timer_misses", health.fmu_timer_misses)
        power_node.setFloat("avionics_vcc", health.avionics_vcc)
        power_node.setFloat("main_vcc", health.main_vcc)
        power_node.setFloat("cell_vcc", health.cell_vcc)
        power_node.setFloat("main_amps", health.main_amps)
        power_node.setInt("total_mah", health.total_mah)
        return health.index

    def pack_payload_dict(self, index):
        row = dict()
        row['timestamp'] = payload_node.getFloat('timestamp')
        row['trigger_num'] = payload_node.getInt('trigger_num')
        return row

    def pack_payload_csv(self, index):
        row = dict()
        row['timestamp'] = '%.4f' % payload_node.getFloat('timestamp')
        row['trigger_num'] = '%d' % payload_node.getInt('trigger_num')
        keys = ['timestamp', 'trigger_num']
        return row, keys

    def unpack_payload_v2(self, buf):
        payload = aura_messages.payload_v2(buf)
        payload_node.setFloat("timestamp", payload.timestamp_sec)
        payload_node.setInt("trigger_num", payload.trigger_num)
        return payload.index

    def unpack_payload_v3(self, buf):
        payload = aura_messages.payload_v3(buf)
        payload_node.setFloat("timestamp", payload.timestamp_sec)
        payload_node.setInt("trigger_num", payload.trigger_num)
        return payload.index

    def pack_event_dict(self, index):
        row = dict()
        timestamp = event_node.getFloat('timestamp')
        if timestamp < 0.001:
            imu_node = getNode('/sensors/imu[0]', True)
            timestamp = imu_node.getFloat('timestamp')
        row['timestamp'] = timestamp
        row['message'] = event_node.getString('message')
        return row

    def pack_event_csv(self, index):
        row = dict()
        row['timestamp'] = '%.4f' % event_node.getFloat('timestamp')
        row['message'] = event_node.getString('message')
        keys = ['timestamp', 'message']
        return row, keys

    def unpack_event_v1(self, buf):
        event = aura_messages.event_v1(buf)
        m = re.match('get: (.*)$', event.message)
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
        event_node.setFloat("timestamp", event.timestamp_sec)
        event_node.setString("message", event.message)
        return event.index

    def unpack_event_v2(self, buf):
        event = aura_messages.event_v2(buf)
        remote_link_node.setInt("sequence_num", event.sequence_num)
        m = re.match('get: (.*)$', event.message)
        if m:
            (prop, value) = m.group(1).split(',')
            # print(prop, value)
            # absolute path
            parts = prop.split('/')
            node_path = '/'.join(parts[0:-1])
            if node_path == '':
                node_path = '/'
            node = getNode(node_path, True)
            name = parts[-1]
            node.setString(name, value)
        event_node.setFloat("timestamp", event.timestamp_sec)
        event_node.setString("message", event.message)
        return 0

packer = Packer()

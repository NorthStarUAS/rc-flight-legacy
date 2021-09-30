import math
import re
import struct

from PropertyTree import PropertyNode

from comms import rc_messages

# FIXME: we are hard coding status flag to zero in many places which
# means we aren't using them properly (and/or wasting bytes)

# FIXME: last_imu_time doesn't address possible multiple imu channels
# (same for other logging categories

ft2m = 0.3048
m2ft = 1.0 / ft2m

r2d = 180.0 / math.pi

kt2mps = 0.5144444444444444444
mps2kt = 1.0 / kt2mps

START_OF_MSG0 = 147
START_OF_MSG1 = 224
    
airdata_node = PropertyNode("/sensors/airdata")
effectors_node = PropertyNode("/effectors")
nav_node = PropertyNode("/filters/filter/0")
gps_node = PropertyNode("/sensors/gps/0")
imu_node = PropertyNode("/sensors/imu/0")
pilot_node = PropertyNode("/sensors/pilot_input")
pos_node = PropertyNode("/position")
pos_pressure_node = PropertyNode("/position/pressure")
pos_combined_node = PropertyNode("/position/combined")
power_node = PropertyNode("/sensors/power")
vel_node = PropertyNode("/velocity")
wind_node = PropertyNode("/filters/wind")
remote_link_node = PropertyNode("/comms/remote_link")
stream_node = PropertyNode("/stream")
switches_node = PropertyNode("/switches")

NUM_ACTUATORS = 8
act_node = PropertyNode("/actuators")

status_node = PropertyNode("/status")
ap_node = PropertyNode("/autopilot")
targets_node = PropertyNode("/autopilot/targets")
tecs_node = PropertyNode("/autopilot/tecs")
task_node = PropertyNode("/task")
route_node = PropertyNode("/task/route")
active_node = PropertyNode("/task/route/active")
home_node = PropertyNode("/task/home")
circle_node = PropertyNode("/task/circle/active")

power_node = PropertyNode("/sensors/power")
payload_node = PropertyNode("/payload")
event_node = PropertyNode("/status/event")

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
        #print("c0 =", c0, "c1 =", c1, i, "[", buf[i], "]")
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
    ap = rc_messages.ap_status_v7()
    act = rc_messages.actuator_v3()
    airdata = rc_messages.airdata_v8()
    nav = rc_messages.nav_v6()
    nav_metrics = rc_messages.nav_metrics_v6()
    gps = rc_messages.gps_v5()
    health = rc_messages.system_health_v6()
    imu = rc_messages.imu_v6()
    incep = rc_messages.inceptors_v1()
    pilot = rc_messages.pilot_v3()
    ap_buf = None
    act_buf = None
    airdata_buf = None
    nav_buf = None
    gps_buf = None
    health_buf = None
    imu_buf = None
    incep_buf = None
    pilot_buf = None
    last_ap_time = -1.0
    last_act_time = -1.0
    last_airdata_time = -1.0
    last_nav_time = -1.0
    last_nav_metrics_time = -1.0
    last_gps_time = -1.0
    last_health_time = -1.0
    last_imu_time = -1.0
    last_incep_time = -1.0
    last_pilot_time = -1.0
    
    def __init__(self):
        pass

    def pack_airdata_bin(self, use_cached=False):
        airdata_time = airdata_node.getDouble("timestamp")
        if not use_cached and airdata_time > self.last_airdata_time:
            self.last_airdata_time = airdata_time
            self.airdata.props2msg(airdata_node)
            self.airdata_buf = self.airdata.pack()
        return self.airdata_buf

    def pack_airdata_dict(self, index):
        airdata_node = PropertyNode("/sensors/airdata")
        row = dict()
        row["millis"] = airdata_node.getUInt("millis")
        row["baro_press_pa"] = airdata_node.getDouble("baro_press_pa")
        row["diff_press_pa"] = airdata_node.getDouble("diff_press_pa")
        row["air_temp_C"] = airdata_node.getDouble("air_temp_C")
        row["airspeed_mps"] = airdata_node.getDouble("airspeed_mps")
        row["altitude_agl_m"] = airdata_node.getDouble("altitude_agl_m")
        row["altitude_true_m"] = airdata_node.getDouble("altitude_true_m")
        row["altitude_ground_m"] = airdata_node.getDouble("altitude_ground_m")
        row["is_airborne"] = airdata_node.getUInt("is_airborne")
        row["flight_timer_millis"] = airdata_node.getUInt("flight_timer_millis")
        row["wind_dir_deg"] = airdata_node.getDouble("wind_dir_deg")
        row["wind_speed_mps"] = airdata_node.getDouble("wind_speed_mps")
        row["pitot_scale_factor"] = airdata_node.getDouble("pitot_scale_factor")
        row["error_count"] = airdata_node.getUInt("error_count")
        #row["tecs_error_total"] = tecs_node.getDouble("error_total")
        #row["tecs_error_diff"] = tecs_node.getDouble("error_diff")
        # print("airdata error:", row["error_count"])
        row["status"] = airdata_node.getInt("status")
        return row

    def unpack_airdata_v6(self, buf):
        air = rc_messages.airdata_v6(buf)

        if air.index > 0:
            print("Warning: airdata index > 0 not supported")
        node = airdata_node

        node.setDouble("timestamp", air.timestamp_sec)
        node.setDouble("pressure_mbar", air.pressure_mbar)
        node.setDouble("temp_C", air.temp_C)
        vel_node.setDouble("airspeed_smoothed_kt", air.airspeed_smoothed_kt)
        if math.isnan(air.altitude_smoothed_m):
            air.altitude_smoothed_m = 0.0
        if math.isnan(air.altitude_true_m):
            air.altitude_true_m = 0.0
        pos_pressure_node.setDouble("altitude_smoothed_m", air.altitude_smoothed_m)
        pos_combined_node.setDouble("altitude_true_m", air.altitude_true_m)
        vel_node.setDouble("pressure_vertical_speed_fps", air.pressure_vertical_speed_fps)
        wind_node.setDouble("wind_dir_deg", air.wind_dir_deg)
        wind_node.setDouble("wind_speed_kt", air.wind_speed_kt)
        wind_node.setDouble("pitot_scale_factor", air.pitot_scale_factor)
        node.setInt("status", air.status)
        return air.index

    def unpack_airdata_v7(self, buf):
        air = rc_messages.airdata_v7(buf)

        if air.index > 0:
            print("Warning: airdata index > 0 not supported")
        node = airdata_node

        node.setDouble("timestamp", air.timestamp_sec)
        node.setDouble("pressure_mbar", air.pressure_mbar)
        node.setDouble("temp_C", air.temp_C)
        vel_node.setDouble("airspeed_smoothed_kt", air.airspeed_smoothed_kt)
        if math.isnan(air.altitude_smoothed_m):
            air.altitude_smoothed_m = 0.0
        if math.isnan(air.altitude_true_m):
            air.altitude_true_m = 0.0
        pos_pressure_node.setDouble("altitude_smoothed_m", air.altitude_smoothed_m)
        pos_combined_node.setDouble("altitude_true_m", air.altitude_true_m)
        vel_node.setDouble("pressure_vertical_speed_fps", air.pressure_vertical_speed_fps)
        wind_node.setDouble("wind_dir_deg", air.wind_dir_deg)
        wind_node.setDouble("wind_speed_kt", air.wind_speed_kt)
        wind_node.setDouble("pitot_scale_factor", air.pitot_scale_factor)
        node.setInt("error_count", air.error_count)
        node.setInt("status", air.status)
        return air.index

    def unpack_airdata_v8(self, buf):
        air = rc_messages.airdata_v8(buf)
        if air.index > 0:
            print("Warning: airdata index > 0 not supported")
        air.msg2props(airdata_node)
        airdata_node.setDouble("timestamp", air.millis / 1000.0)
        pos_node.setDouble("altitude_ground_m", air.altitude_ground_m)
        return air.index

    # FIXME: think about how we are dealing with skips and gps lower rate?
    def pack_gps_bin(self, use_cached=False):
        gps_time = gps_node.getDouble("timestamp")
        if use_cached:
            return self.gps_buf
        elif (gps_time > self.last_gps_time) or self.gps_buf is None:
            self.last_gps_time = gps_time
            self.gps.props2msg(gps_node)
            self.gps_buf = self.gps.pack()
            return self.gps_buf
        else:
            return None

    def pack_gps_dict(self, index):
        gps_node = PropertyNode("/sensors/gps/%d" % index)
        row = dict()
        row["index"] = gps_node.getUInt("index")
        row["millis"] = gps_node.getUInt("millis")
        row["unix_usec"] = gps_node.getUInt64("unix_usec")
        row["num_sats"] = gps_node.getUInt("num_sats")
        row["status"] = gps_node.getUInt("status")
        row["longitude_raw"] = gps_node.getInt("longitude_raw")
        row["latitude_raw"] = gps_node.getInt("latitude_raw")
        row["altitude_m"] = gps_node.getDouble("altitude_m")
        row["vn_mps"] = gps_node.getDouble("vn_mps")
        row["ve_mps"] = gps_node.getDouble("ve_mps")
        row["vd_mps"] = gps_node.getDouble("vd_mps")
        row["hAcc_m"] = gps_node.getDouble("hAcc_m")
        row["vAcc_m"] = gps_node.getDouble("vAcc_m")
        row["hdop"] = gps_node.getDouble("hdop")
        row["vdop"] = gps_node.getDouble("vdop")
        return row

    def unpack_gps_v3(self, buf):
        gps = rc_messages.gps_v3(buf)

        if gps.index > 0:
            print("Warning: gps index > 0 not supported")
        node = gps_node

        node.setDouble("timestamp", gps.timestamp_sec)
        node.setDouble("latitude_deg", gps.latitude_deg)
        node.setDouble("longitude_deg", gps.longitude_deg)
        node.setDouble("altitude_m", gps.altitude_m)
        node.setDouble("vn_ms", gps.vn_ms)
        node.setDouble("ve_ms", gps.ve_ms)
        node.setDouble("vd_ms", gps.vd_ms)
        node.setDouble("unix_time_sec", gps.unixtime_sec)
        node.setInt("satellites", gps.satellites)
        node.setDouble("horiz_accuracy_m", gps.horiz_accuracy_m)
        node.setDouble("vert_accuracy_m", gps.vert_accuracy_m)
        node.setDouble("pdop", gps.pdop)
        node.setInt("fixType", gps.fix_type)
        node.setInt("status", 0)
        return gps.index

    def unpack_gps_v4(self, buf):
        gps = rc_messages.gps_v4(buf)

        if gps.index > 0:
            print("Warning: gps index > 0 not supported")
        node = gps_node

        node.setDouble("timestamp", gps.timestamp_sec)
        node.setDouble("latitude_deg", gps.latitude_deg)
        node.setDouble("longitude_deg", gps.longitude_deg)
        node.setDouble("altitude_m", gps.altitude_m)
        node.setDouble("vn_ms", gps.vn_ms)
        node.setDouble("ve_ms", gps.ve_ms)
        node.setDouble("vd_ms", gps.vd_ms)
        node.setDouble("unix_time_sec", gps.unixtime_sec)
        node.setInt("satellites", gps.satellites)
        node.setDouble("horiz_accuracy_m", gps.horiz_accuracy_m)
        node.setDouble("vert_accuracy_m", gps.vert_accuracy_m)
        node.setDouble("pdop", gps.pdop)
        node.setInt("fixType", gps.fix_type)
        node.setInt("status", 0)
        return gps.index

    def unpack_gps_v5(self, buf):
        gps = rc_messages.gps_v5(buf)
        if gps.index > 0:
            print("Warning: gps index > 0 not supported")
        gps.msg2props(gps_node)
        gps_node.setDouble("timestamp", gps.millis / 1000.0)
        gps_node.setDouble("unix_time_sec", gps.unix_usec / 1000000.0)
        gps_node.setDouble("latitude_deg", gps.latitude_raw / 10000000.0 )
        gps_node.setDouble("longitude_deg", gps.longitude_raw / 10000000.0)
        return gps.index

    # only support primary imu for now
    def pack_imu_bin(self, use_cached=False):
        imu_time = imu_node.getDouble("timestamp")
        if not use_cached and imu_time > self.last_imu_time:
            self.last_imu_time = imu_time
            self.imu.props2msg(imu_node)
            self.imu_buf = self.imu.pack()
        return self.imu_buf

    def pack_imu_dict(self, index):
        imu_node = PropertyNode("/sensors/imu/%d" % index)
        row = dict()
        row["index"] = imu_node.getUInt("index")
        row["millis"] = imu_node.getUInt("millis")
        row["ax_raw"] = imu_node.getDouble("ax_raw")
        row["ay_raw"] = imu_node.getDouble("ay_raw")
        row["az_raw"] = imu_node.getDouble("az_raw")
        row["hx_raw"] = imu_node.getDouble("hx_raw")
        row["hy_raw"] = imu_node.getDouble("hy_raw")
        row["hz_raw"] = imu_node.getDouble("hz_raw")
        row["ax_mps2"] = imu_node.getDouble("ax_mps2")
        row["ay_mps2"] = imu_node.getDouble("ay_mps2")
        row["az_mps2"] = imu_node.getDouble("az_mps2")
        row["p_rps"] = imu_node.getDouble("p_rps")
        row["q_rps"] = imu_node.getDouble("q_rps")
        row["r_rps"] = imu_node.getDouble("r_rps")
        row["hx"] = imu_node.getDouble("hx")
        row["hy"] = imu_node.getDouble("hy")
        row["hz"] = imu_node.getDouble("hz")
        row["temp_C"] = imu_node.getDouble("temp_C")
        return row

    def unpack_imu_v4(self, buf):
        imu = rc_messages.imu_v4(buf)

        if imu.index > 0:
            print("Warning: imu index > 0 not supported")
        node = imu_node

        node.setDouble("timestamp", imu.timestamp_sec)
        node.setDouble("p_rps", imu.p_rad_sec)
        node.setDouble("q_rps", imu.q_rad_sec)
        node.setDouble("r_rps", imu.r_rad_sec)
        node.setDouble("ax_mps2", imu.ax_mps_sec)
        node.setDouble("ay_mps2", imu.ay_mps_sec)
        node.setDouble("az_mps2", imu.az_mps_sec)
        node.setDouble("hx", imu.hx)
        node.setDouble("hy", imu.hy)
        node.setDouble("hz", imu.hz)
        node.setDouble("temp_C", imu.temp_C)
        node.setInt("status", imu.status)
        return imu.index

    def unpack_imu_v5(self, buf):
        imu = rc_messages.imu_v5(buf)

        if imu.index > 0:
            print("Warning: imu index > 0 not supported")
        node = imu_node

        node.setDouble("timestamp", imu.timestamp_sec)
        node.setDouble("p_rps", imu.p_rad_sec)
        node.setDouble("q_rps", imu.q_rad_sec)
        node.setDouble("r_rps", imu.r_rad_sec)
        node.setDouble("ax_mps2", imu.ax_mps_sec)
        node.setDouble("ay_mps2", imu.ay_mps_sec)
        node.setDouble("az_mps2", imu.az_mps_sec)
        node.setDouble("hx", imu.hx)
        node.setDouble("hy", imu.hy)
        node.setDouble("hz", imu.hz)
        node.setDouble("ax_raw", imu.ax_raw)
        node.setDouble("ay_raw", imu.ay_raw)
        node.setDouble("az_raw", imu.az_raw)
        node.setDouble("hx_raw", imu.hx_raw)
        node.setDouble("hy_raw", imu.hy_raw)
        node.setDouble("hz_raw", imu.hz_raw)
        node.setDouble("temp_C", imu.temp_C)
        node.setInt("status", imu.status)
        return imu.index

    def unpack_imu_v6(self, buf):
        imu = rc_messages.imu_v6(buf)
        if imu.index > 0:
            print("Warning: imu index > 0 not supported")
        imu.msg2props(imu_node)
        imu_node.setDouble("timestamp", imu.millis / 1000.0)
        return imu.index

    def pack_nav_bin(self, use_cached=False):
        nav_time = nav_node.getDouble("timestamp")
        if (not use_cached and nav_time > self.last_nav_time) or self.nav_buf is None:
            self.last_nav_time = nav_time
            self.nav.props2msg(nav_node)
            self.nav_buf = self.nav.pack()
        return self.nav_buf

    def pack_nav_dict(self, index):
        nav_node = PropertyNode("/filters/filter/%d" % index)
        row = dict()
        row["index"] = nav_node.getUInt("index")
        row["millis"] = nav_node.getUInt("millis")
        row["latitude_raw"] = nav_node.getInt("latitude_raw")
        row["longitude_raw"] = nav_node.getInt("longitude_raw")
        row["altitude_m"] = nav_node.getDouble("altitude_m")
        row["vn_mps"] = nav_node.getDouble("vn_mps")
        row["ve_mps"] = nav_node.getDouble("ve_mps")
        row["vd_mps"] = nav_node.getDouble("vd_mps")
        row["roll_deg"] = nav_node.getDouble("roll_deg")
        row["pitch_deg"] = nav_node.getDouble("pitch_deg")
        row["yaw_deg"] = nav_node.getDouble("yaw_deg")
        row["sequence_num"] = nav_node.getUInt("sequence_num")
        row["status"] = nav_node.getUInt("status")
        return row
    
    def unpack_filter_v4(self, buf):
        nav = rc_messages.filter_v4(buf)

        if nav.index > 0:
            print("Warning: nav index > 0 not supported")
        node = nav_node

        node.setDouble("timestamp", nav.timestamp_sec)
        node.setDouble("latitude_deg", nav.latitude_deg)
        node.setDouble("longitude_deg", nav.longitude_deg)
        node.setDouble("altitude_m", nav.altitude_m)
        node.setDouble("vn_ms", nav.vn_ms)
        node.setDouble("ve_ms", nav.ve_ms)
        node.setDouble("vd_ms", nav.vd_ms)
        node.setDouble("roll_deg", nav.roll_deg)
        node.setDouble("pitch_deg", nav.pitch_deg)
        node.setDouble("heading_deg", nav.yaw_deg)
        node.setDouble("p_bias", nav.p_bias)
        node.setDouble("q_bias", nav.q_bias)
        node.setDouble("r_bias", nav.r_bias)
        node.setDouble("ax_bias", nav.ax_bias)
        node.setDouble("ay_bias", nav.ay_bias)
        node.setDouble("az_bias", nav.az_bias)
        if nav.sequence_num >= 1:
            remote_link_node.setInt("sequence_num", nav.sequence_num)
        node.setInt("status", nav.status)

        return nav.index

    def unpack_filter_v5(self, buf):
        nav = rc_messages.filter_v5(buf)

        if nav.index > 0:
            print("Warning: nav index > 0 not supported")
        node = nav_node

        node.setDouble("timestamp", nav.timestamp_sec)
        node.setDouble("latitude_deg", nav.latitude_deg)
        node.setDouble("longitude_deg", nav.longitude_deg)
        node.setDouble("altitude_m", nav.altitude_m)
        node.setDouble("vn_ms", nav.vn_ms)
        node.setDouble("ve_ms", nav.ve_ms)
        node.setDouble("vd_ms", nav.vd_ms)
        node.setDouble("roll_deg", nav.roll_deg)
        node.setDouble("pitch_deg", nav.pitch_deg)
        node.setDouble("heading_deg", nav.yaw_deg)
        node.setDouble("p_bias", nav.p_bias)
        node.setDouble("q_bias", nav.q_bias)
        node.setDouble("r_bias", nav.r_bias)
        node.setDouble("ax_bias", nav.ax_bias)
        node.setDouble("ay_bias", nav.ay_bias)
        node.setDouble("az_bias", nav.az_bias)
        node.setDouble("max_pos_cov", nav.max_pos_cov)
        node.setDouble("max_vel_cov", nav.max_vel_cov)
        node.setDouble("max_att_cov", nav.max_att_cov)
        if nav.sequence_num >= 1:
            remote_link_node.setInt("sequence_num", nav.sequence_num)
        node.setInt("status", nav.status)

        return nav.index

    def unpack_nav_v6(self, buf):
        nav = rc_messages.nav_v6(buf)
        if nav.index > 0:
            print("Warning: nav index > 0 not supported")
        nav.msg2props(nav_node)
        nav_node.setDouble("timestamp", nav.millis / 1000.0)
        nav_node.setDouble("latitude_deg", nav.latitude_raw / 10000000.0 )
        nav_node.setDouble("longitude_deg", nav.longitude_raw / 10000000.0)
        if nav.sequence_num >= 1:
            remote_link_node.setInt("sequence_num", nav.sequence_num)
        # FIXME: should this code be here?
        nav_node.setDouble( "groundtrack_deg",
                            90 - math.atan2(nav.vn_mps, nav.ve_mps) * r2d )
        gs_mps = sqrt(nav.vn_mps*nav.vn_mps + nav.ve_mps*nav.ve_mps)
        nav_node.setDouble( "groundspeed_ms", gs_mps )
        nav_node.setDouble( "groundspeed_kt", gs_mps * mps2kt )
        return nav.index

    def pack_nav_metrics_bin(self, use_cached=False):
        nav_metrics_time = nav_node.getDouble("timestamp")
        if (not use_cached and nav_metrics_time > self.last_nav_metrics_time) or self.nav_metrics_buf is None:
            self.last_nav_metrics_time = nav_metrics_time
            self.nav_metrics.props2msg(nav_node)
            self.nav_metrics_buf = self.nav_metrics.pack()
        return self.nav_metrics_buf

    # fixme: do nav metrics dict message

    def unpack_nav_metrics_v6(self, buf):
        metrics = rc_messages.nav_metrics_v6(buf)
        if metrics.index > 0:
            print("Warning: nav metrics index > 0 not supported")
        metrics.msg2props(nav_node)
        return metrics.index

    # fixme: effectors?
    
    def pack_act_bin(self, use_cached=False):
        act_time = act_node.getDouble('timestamp')
        if not use_cached and act_time > self.last_act_time:
            self.last_act_time = act_time
            self.act.index = 0
            self.act.timestamp_sec = act_time
            self.act.aileron = act_node.getDouble("aileron")
            self.act.elevator = act_node.getDouble("elevator")
            self.act.throttle = act_node.getDouble("throttle")
            self.act.rudder = act_node.getDouble("rudder")
            self.act.channel5 = act_node.getDouble("channel5")
            self.act.flaps = act_node.getDouble("flaps")
            self.act.channel7 = act_node.getDouble("channel7")
            self.act.channel8 = act_node.getDouble("channel8")
            self.act.status = 0
            self.act_buf = self.act.pack()
        return self.act_buf

    def pack_act_dict(self, index):
        row = dict()
        row['timestamp'] = act_node.getDouble('timestamp')
        row['aileron_norm'] = act_node.getDouble('aileron')
        row['elevator_norm'] = act_node.getDouble('elevator')
        row['throttle_norm'] = act_node.getDouble('throttle')
        row['rudder_norm'] = act_node.getDouble('rudder')
        row['channel5_norm'] = act_node.getDouble('channel5')
        row['flaps_norm'] = act_node.getDouble('flaps')
        row['channel7_norm'] = act_node.getDouble('channel7')
        row['channel8_norm'] = act_node.getDouble('channel8')
        row['status'] = act_node.getInt('status')
        return row

    def unpack_act_v2(self, buf):
        act = rc_messages.actuator_v2(buf)
        act_node.setDouble("timestamp", act.timestamp_sec)
        act_node.setDouble("aileron", act.aileron)
        act_node.setDouble("elevator", act.elevator)
        act_node.setDouble("throttle", act.throttle)
        act_node.setDouble("rudder", act.rudder)
        act_node.setDouble("channel5", act.channel5)
        act_node.setDouble("flaps", act.flaps)
        act_node.setDouble("channel7", act.channel7)
        act_node.setDouble("channel8", act.channel8)
        act_node.setInt("status", act.status)
        return act.index

    def unpack_act_v3(self, buf):
        act = rc_messages.actuator_v3(buf)
        act_node.setDouble("timestamp", act.timestamp_sec)
        act_node.setDouble("aileron", act.aileron)
        act_node.setDouble("elevator", act.elevator)
        act_node.setDouble("throttle", act.throttle)
        act_node.setDouble("rudder", act.rudder)
        act_node.setDouble("channel5", act.channel5)
        act_node.setDouble("flaps", act.flaps)
        act_node.setDouble("channel7", act.channel7)
        act_node.setDouble("channel8", act.channel8)
        act_node.setInt("status", act.status)
        return act.index

    def unpack_effectors_v1(self, buf):
        eff = rc_messages.effectors_v1(buf)
        eff.msg2props(effectors_node)
        effectors_node.setDouble("aileron", eff.channel[1])
        effectors_node.setDouble("elevator", eff.channel[2])
        effectors_node.setDouble("throttle", eff.channel[0])
        effectors_node.setDouble("rudder", eff.channel[3])
        effectors_node.setDouble("flaps", eff.channel[4])
        effectors_node.setDouble("gear", eff.channel[5])
        return eff.index

    def pack_inceptors_bin(self, use_cached=False):
        incep_time = pilot_node.getDouble("timestamp")
        if not use_cached and incep_time > self.last_incep_time:
            self.last_incep_time = incep_time
            self.incep.props2msg(pilot_node)
            self.incep_buf = self.incep.pack()
        return self.incep_buf

    def pack_pilot_dict(self, index):
        pilot_node = PropertyNode('/sensors/pilot_input')
        row = dict()
        row['timestamp'] = pilot_node.getDouble('timestamp')
        row['channel[0]'] = pilot_node.getDouble('channel', 0)
        row['channel[1]'] = pilot_node.getDouble('channel', 1)
        row['channel[2]'] = pilot_node.getDouble('channel', 2)
        row['channel[3]'] = pilot_node.getDouble('channel', 3)
        row['channel[4]'] = pilot_node.getDouble('channel', 4)
        row['channel[5]'] = pilot_node.getDouble('channel', 5)
        row['channel[6]'] = pilot_node.getDouble('channel', 6)
        row['channel[7]'] = pilot_node.getDouble('channel', 7)
        row['status'] = pilot_node.getInt('status')
        return row

    def unpack_pilot_v2(self, buf):
        pilot = rc_messages.pilot_v2(buf)

        if pilot.index > 0:
            print("Warning: pilot index > 0 not supported")

        pilot_node.setDouble("timestamp", pilot.timestamp_sec)
        pilot_node.setDouble("channel", pilot.channel[0], 0)
        pilot_node.setDouble("channel", pilot.channel[1], 1)
        pilot_node.setDouble("channel", pilot.channel[2], 2)
        pilot_node.setDouble("channel", pilot.channel[3], 3)
        pilot_node.setDouble("channel", pilot.channel[4], 4)
        pilot_node.setDouble("channel", pilot.channel[5], 5)
        pilot_node.setDouble("channel", pilot.channel[6], 6)
        pilot_node.setDouble("channel", pilot.channel[7], 7)
        pilot_node.setInt("status", pilot.status)

        return pilot.index

    def unpack_pilot_v3(self, buf):
        pilot = rc_messages.pilot_v3(buf)

        if pilot.index > 0:
            print("Warning: pilot index > 0 not supported")

        pilot_node.setDouble("timestamp", pilot.timestamp_sec)
        pilot_node.setDouble("channel", pilot.channel[0], 0)
        pilot_node.setDouble("channel", pilot.channel[1], 1)
        pilot_node.setDouble("channel", pilot.channel[2], 2)
        pilot_node.setDouble("channel", pilot.channel[3], 3)
        pilot_node.setDouble("channel", pilot.channel[4], 4)
        pilot_node.setDouble("channel", pilot.channel[5], 5)
        pilot_node.setDouble("channel", pilot.channel[6], 6)
        pilot_node.setDouble("channel", pilot.channel[7], 7)
        pilot_node.setInt("status", pilot.status)

        return pilot.index

    def unpack_pilot_v4(self, buf):
        pilot = rc_messages.pilot_v4(buf)
        if pilot.index > 0:
            print("Warning: pilot index > 0 not supported")
        pilot.msg2props(pilot_node)
        pilot_node.setDouble("timestamp", pilot.millis / 1000.0)
        switches_node.setBool("master_switch", pilot.master_switch)
        switches_node.setBool("throttle_safety", pilot.throttle_safety)

        return pilot.index

    def unpack_power_v1(self, buf):
        power = rc_messages.power_v1(buf)
        if power.index > 0:
            print("Warning: power index > 0 not supported")
        power.msg2props(power_node)
        power_node.setDouble("timestamp", power.millis / 1000.0)
        return power.index

    def pack_ap_status_bin(self, use_cached=False):
        ap_time = status_node.getDouble("frame_time")
        if not use_cached and ap_time > self.last_ap_time:
            self.last_ap_time = ap_time
            self.ap.index = 0
            self.ap.timestamp_sec = ap_time
            # status flags (up to 8 could be supported)
            self.ap.flags = 0
            if ap_node.getBool("master_switch"):
                self.ap.flags += 1 # |= (1 << 0)
            self.ap.groundtrack_deg = targets_node.getDouble("groundtrack_deg")
            self.ap.roll_deg = targets_node.getDouble("roll_deg")
            target_agl_ft = targets_node.getDouble("altitude_agl_ft")
            ground_m = pos_node.getDouble("altitude_ground_m")
            # if pressure based:
            #   error_m = pos_pressure_node.getDouble("pressure_error_m")
            #   target_msl_ft = (ground_m + error_m) * m2ft + target_agl_ft
            # else: ...
            self.ap.altitude_msl_ft = ground_m * m2ft + target_agl_ft
            self.ap.altitude_ground_m = ground_m
            self.ap.pitch_deg = targets_node.getDouble("pitch_deg")
            self.ap.airspeed_kt = targets_node.getDouble("airspeed_kt")
            self.ap.flight_timer = task_node.getDouble("flight_timer")
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
                wp_path = "wpt/%d" % self.ap.wp_index
                wp_node = active_node.getChild(wp_path)
                self.ap.wp_longitude_deg = wp_node.getDouble("longitude_deg")
                self.ap.wp_latitude_deg = wp_node.getDouble("latitude_deg")
            elif counter == self.ap.route_size:
                self.ap.wp_longitude_deg = circle_node.getDouble("longitude_deg")
                self.ap.wp_latitude_deg = circle_node.getDouble("latitude_deg")
                self.ap.wp_index = 65534
                self.ap.task_attribute = int(round(circle_node.getDouble("radius_m") * 10))
                if self.ap.task_attribute > 32767: self.ap.task_attribute = 32767
            elif counter == self.ap.route_size + 1:
                self.ap.wp_longitude_deg = home_node.getDouble("longitude_deg")
                self.ap.wp_latitude_deg = home_node.getDouble("latitude_deg")
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
        row['timestamp'] = targets_node.getDouble('timestamp')
        row['master_switch'] = ap_node.getBool("master_switch")
        row['groundtrack_deg'] = targets_node.getDouble('groundtrack_deg')
        row['roll_deg'] = targets_node.getDouble('roll_deg')
        row['altitude_msl_ft'] = targets_node.getDouble('altitude_msl_ft')
        row['pitch_deg'] = targets_node.getDouble('pitch_deg')
        row['airspeed_kt'] = targets_node.getDouble('airspeed_kt')
        row['altitude_ground_m'] = pos_node.getDouble("altitude_ground_m")
        row['tecs_target_tot'] = tecs_node.getDouble("target_total")
        row['flight_timer'] = task_node.getDouble("flight_timer")
        row['target_waypoint_idx'] = route_node.getInt("target_waypoint_idx")
        route_size = active_node.getInt("route_size")
        row['route_size'] = route_size
        row['task_attribute'] = 0.0
        if self.wp_counter < route_size:
            wp_node = active_node.getChild('wpt/%d' % self.wp_counter)
            row['wpt_index'] = self.wp_counter
            row['wpt_longitude_deg'] = wp_node.getDouble("longitude_deg")
            row['wpt_latitude_deg'] = wp_node.getDouble("latitude_deg")
        elif self.wp_counter == route_size:
            row['wpt_index'] = 65534
            row['wpt_longitude_deg'] = circle_node.getDouble("longitude_deg")
            row['wpt_latitude_deg'] = circle_node.getDouble("latitude_deg")
            row['task_attribute'] = int(round(circle_node.getDouble("radius_m") * 10))
        elif self.wp_counter == route_size + 1:
            row['wpt_index'] = 65535
            row['wpt_longitude_deg'] = home_node.getDouble("longitude_deg")
            row['wpt_latitude_deg'] = home_node.getDouble("latitude_deg")
        row['current_task'] = task_node.getString("current_task")
        self.wp_counter += 1
        if self.wp_counter >= route_size + 2:
            self.wp_counter = 0
        return row

    def unpack_ap_status_v6(self, buf):
        ap = rc_messages.ap_status_v6(buf)

        index = ap.index

        wp_lon = ap.wp_longitude_deg
        wp_lat = ap.wp_latitude_deg
        wp_index = ap.wp_index
        route_size = ap.route_size
        task_id = ap.task_id
        task_attrib = ap.task_attribute

        targets_node.setDouble("timestamp", ap.timestamp_sec)
        flags = ap.flags
        ap_node.setBool("master_switch", flags & (1<<0))
        ap_node.setBool("pilot_pass_through", flags & (1<<1))
        targets_node.setDouble("groundtrack_deg", ap.groundtrack_deg)
        targets_node.setDouble("roll_deg", ap.roll_deg)
        targets_node.setDouble("altitude_msl_ft", ap.altitude_msl_ft)
        pos_node.setDouble("altitude_ground_m", ap.altitude_ground_m)
        targets_node.setDouble("pitch_deg", ap.pitch_deg)
        targets_node.setDouble("airspeed_kt", ap.airspeed_kt)
        status_node.setDouble("flight_timer", ap.flight_timer)
        status_node.setBool("onboard_flight_timer", True)
        if route_size != active_node.getInt("route_size"):
            # route size change, zero all the waypoint coordinates
            for i in range(active_node.getInt("route_size")):
                wp_node = active_node.getChild('wpt/%d' % i)
                wp_node.setDouble("longitude_deg", 0)
                wp_node.setDouble("latitude_deg", 0)
        route_node.setInt("target_waypoint_idx", ap.target_waypoint_idx)
        if wp_index < route_size:
            wp_node = active_node.getChild('wpt/%d' % wp_index)
            wp_node.setDouble("longitude_deg", wp_lon)
            wp_node.setDouble("latitude_deg", wp_lat)
        elif wp_index == 65534:
            circle_node.setDouble("longitude_deg", wp_lon)
            circle_node.setDouble("latitude_deg", wp_lat)
            circle_node.setDouble("radius_m", task_attrib / 10.0)
        elif wp_index == 65535:
            home_node.setDouble("longitude_deg", wp_lon)
            home_node.setDouble("latitude_deg", wp_lat)
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
        ap = rc_messages.ap_status_v7(buf)

        index = ap.index

        wp_lon = ap.wp_longitude_deg
        wp_lat = ap.wp_latitude_deg
        wp_index = ap.wp_index
        route_size = ap.route_size
        task_id = ap.task_id
        task_attrib = ap.task_attribute

        targets_node.setDouble("timestamp", ap.timestamp_sec)
        flags = ap.flags
        ap_node.setBool("master_switch", flags & (1<<0))
        ap_node.setBool("pilot_pass_through", flags & (1<<1))
        targets_node.setDouble("groundtrack_deg", ap.groundtrack_deg)
        targets_node.setDouble("roll_deg", ap.roll_deg)
        targets_node.setDouble("altitude_msl_ft", ap.altitude_msl_ft)
        pos_node.setDouble("altitude_ground_m", ap.altitude_ground_m)
        targets_node.setDouble("pitch_deg", ap.pitch_deg)
        targets_node.setDouble("airspeed_kt", ap.airspeed_kt)
        status_node.setDouble("flight_timer", ap.flight_timer)
        status_node.setBool("onboard_flight_timer", True)
        if route_size != active_node.getInt("route_size"):
            # route size change, zero all the waypoint coordinates
            for i in range(active_node.getInt("route_size")):
                wp_node = active_node.getChild('wpt/%d' % i)
                wp_node.setDouble("longitude_deg", 0)
                wp_node.setDouble("latitude_deg", 0)
        route_node.setInt("target_waypoint_idx", ap.target_waypoint_idx)
        if wp_index < route_size:
            wp_node = active_node.getChild('wpt/%d' % wp_index)
            wp_node.setDouble("longitude_deg", wp_lon)
            wp_node.setDouble("latitude_deg", wp_lat)
        elif wp_index == 65534:
            circle_node.setDouble("longitude_deg", wp_lon)
            circle_node.setDouble("latitude_deg", wp_lat)
            circle_node.setDouble("radius_m", task_attrib / 10.0)
        elif wp_index == 65535:
            home_node.setDouble("longitude_deg", wp_lon)
            home_node.setDouble("latitude_deg", wp_lat)
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
            elif task_attrib == 9:
                task_node.setString("calib_message", "finished successfully!")
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

    def unpack_ap_targets_v1(self, buf):
        ap = rc_messages.ap_targets_v1(buf)
        ap.msg2props(targets_node)
        return 0

    def unpack_mission_v1(self, buf):
        mission = rc_messages.mission_v1(buf)

        index = mission.index

        wp_lon = mission.wp_longitude_raw / 10000000.0
        wp_lat = mission.wp_latitude_raw / 10000000.0
        wp_index = mission.wp_index
        route_size = mission.route_size
        status_node.setDouble("flight_timer", mission.flight_timer)
        status_node.setBool("onboard_flight_timer", True)
        if route_size != active_node.getInt("route_size"):
            # route size change, zero all the waypoint coordinates
            for i in range(active_node.getInt("route_size")):
                wp_node = active_node.getChild('wpt/%d' % i)
                wp_node.setDouble("longitude_deg", 0)
                wp_node.setDouble("latitude_deg", 0)
        route_node.setInt("target_waypoint_idx", mission.target_waypoint_idx)
        if wp_index < route_size:
            wp_node = active_node.getChild('wpt/%d' % wp_index)
            wp_node.setDouble("longitude_deg", wp_lon)
            wp_node.setDouble("latitude_deg", wp_lat)
        elif wp_index == 65534:
            circle_node.setDouble("longitude_deg", wp_lon)
            circle_node.setDouble("latitude_deg", wp_lat)
            circle_node.setDouble("radius_m", mission.task_attribute / 10.0)
        elif wp_index == 65535:
            home_node.setDouble("longitude_deg", wp_lon)
            home_node.setDouble("latitude_deg", wp_lat)
        task_node.setString("current_task", mission.task_name)

        active_node.setInt("route_size", route_size)

        return index

    def pack_system_health_bin(self, use_cached=False):
        health_time = status_node.getDouble('frame_time')
        if not use_cached and health_time > self.last_health_time:
            self.last_health_time = health_time
            self.health.index = 0
            self.health.timestamp_sec = health_time
            self.health.system_load_avg = status_node.getDouble("system_load_avg")
            self.health.fmu_timer_misses = status_node.getInt("fmu_timer_misses")
            self.health.avionics_vcc = power_node.getDouble("avionics_vcc")
            self.health.main_vcc = power_node.getDouble("main_vcc")
            self.health.cell_vcc = power_node.getDouble("cell_vcc")
            self.health.main_amps = power_node.getDouble("main_amps")
            self.health.total_mah = power_node.getDouble("total_mah")
            self.health_buf = self.health.pack()
        return self.health_buf

    def pack_system_health_dict(self, index):
        row = dict()
        row['timestamp'] = status_node.getDouble('frame_time')
        row['system_load_avg'] = status_node.getDouble('system_load_avg')
        row['fmu_timer_misses'] = status_node.getDouble('fmu_timer_misses')
        row['avionics_vcc'] = power_node.getDouble('avionics_vcc')
        row['main_vcc'] = power_node.getDouble('main_vcc')
        row['cell_vcc'] = power_node.getDouble('cell_vcc')
        row['main_amps'] = power_node.getDouble('main_amps')
        row['total_mah'] = power_node.getDouble('total_mah')
        return row

    def unpack_system_health_v4(self, buf):
        health = rc_messages.system_health_v4(buf)
        status_node.setDouble("frame_time", health.timestamp_sec)
        status_node.setDouble("system_load_avg", health.system_load_avg)
        power_node.setDouble("avionics_vcc", health.avionics_vcc)
        power_node.setDouble("main_vcc", health.main_vcc)
        power_node.setDouble("cell_vcc", health.cell_vcc)
        power_node.setDouble("main_amps", health.main_amps)
        power_node.setInt("total_mah", health.total_mah)
        return health.index

    def unpack_system_health_v5(self, buf):
        health = rc_messages.system_health_v5(buf)
        status_node.setDouble("frame_time", health.timestamp_sec)
        status_node.setDouble("system_load_avg", health.system_load_avg)
        power_node.setDouble("avionics_vcc", health.avionics_vcc)
        power_node.setDouble("main_vcc", health.main_vcc)
        power_node.setDouble("cell_vcc", health.cell_vcc)
        power_node.setDouble("main_amps", health.main_amps)
        power_node.setInt("total_mah", int(health.total_mah))
        return health.index

    def unpack_system_health_v6(self, buf):
        health = rc_messages.system_health_v6(buf)
        status_node.setDouble("frame_time", health.timestamp_sec)
        status_node.setDouble("system_load_avg", health.system_load_avg)
        status_node.setInt("fmu_timer_misses", health.fmu_timer_misses)
        power_node.setDouble("avionics_vcc", health.avionics_vcc)
        power_node.setDouble("main_vcc", health.main_vcc)
        power_node.setDouble("cell_vcc", health.cell_vcc)
        power_node.setDouble("main_amps", health.main_amps)
        power_node.setInt("total_mah", int(health.total_mah))
        return health.index

    def unpack_status_v7(self, buf):
        status = rc_messages.status_v7(buf)
        if status.index > 0:
            print("Warning: status index > 0 not supported")
        status.msg2props(status_node)
        status_node.setDouble("timestamp", status.millis / 1000.0)
        return status.index

        status_node.setDouble("frame_time", health.timestamp_sec)
        status_node.setDouble("system_load_avg", health.system_load_avg)
        status_node.setInt("fmu_timer_misses", health.fmu_timer_misses)
        power_node.setDouble("avionics_vcc", health.avionics_vcc)
        power_node.setDouble("main_vcc", health.main_vcc)
        power_node.setDouble("cell_vcc", health.cell_vcc)
        power_node.setDouble("main_amps", health.main_amps)
        power_node.setInt("total_mah", int(health.total_mah))
        return health.index

    def pack_payload_dict(self, index):
        row = dict()
        row['timestamp'] = payload_node.getDouble('timestamp')
        row['trigger_num'] = payload_node.getInt('trigger_num')
        return row

    def unpack_payload_v2(self, buf):
        payload = rc_messages.payload_v2(buf)
        payload_node.setDouble("timestamp", payload.timestamp_sec)
        payload_node.setInt("trigger_num", payload.trigger_num)
        return payload.index

    def unpack_payload_v3(self, buf):
        payload = rc_messages.payload_v3(buf)
        payload_node.setDouble("timestamp", payload.timestamp_sec)
        payload_node.setInt("trigger_num", payload.trigger_num)
        return payload.index

    def pack_event_dict(self, index):
        row = dict()
        timestamp = event_node.getDouble('timestamp')
        if timestamp < 0.001:
            imu_node = PropertyNode('/sensors/imu/0')
            timestamp = imu_node.getDouble('timestamp')
        row['timestamp'] = timestamp
        row['message'] = event_node.getString('message')
        return row

    def unpack_event_v1(self, buf):
        event = rc_messages.event_v1(buf)
        m = re.match('get: (.*)$', event.message)
        if m:
            (prop, value) = m.group(1).split(',')
            # print prop, value
            # absolute path
            parts = prop.split('/')
            node_path = '/'.join(parts[0:-1])
            if node_path == '':
                node_path = '/'
            node = PropertyNode(node_path)
            name = parts[-1]
            node.setString(name, value)
        event_node.setDouble("timestamp", event.timestamp_sec)
        event_node.setString("message", event.message)
        return event.index

    def unpack_event_v2(self, buf):
        event = rc_messages.event_v2(buf)
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
            node = PropertyNode(node_path)
            name = parts[-1]
            node.setString(name, value)
        event_node.setDouble("timestamp", event.timestamp_sec)
        event_node.setString("message", event.message)
        return 0

    def unpack_command_v1(self, buf):
        command = rc_messages.command_v1(buf)
        pos1 = command.message.find(" ")
        pos2 = command.message.find(" ", pos1+1)
        path = command.message[pos1+1:pos2]
        json = command.message[pos2+1:len(command.message)]
        print(path)
        print(json)
        node = PropertyNode(path)
        if not node.set_json_string(json):
            print("json string parsing/setting failed")
        return 0

    def unpack_ack_v1(self, buf):
        ack = rc_messages.ack_v1(buf)
        if ack.result > 0:
            remote_link_node.setInt("sequence_num", ack.sequence_num)
        return 0

packer = Packer()

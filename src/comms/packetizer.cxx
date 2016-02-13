#include "python/pyprops.hxx"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

#include "include/globaldefs.h"
#include "util/timing.h"

#include "packetizer.hxx"


UGPacketizer::UGPacketizer() {
    gps_node = pyGetNode("/sensors/gps", true);
    imu_node = pyGetNode("/sensors/imu", true);
    airdata_node = pyGetNode("/sensors/airdata", true);
    pos_node = pyGetNode("/position", true);
    pos_pressure_node = pyGetNode("/position/pressure", true);
    pos_combined_node = pyGetNode("/position/combined", true);
    vel_node = pyGetNode("/velocity", true);
    filter_node = pyGetNode("/filters/filter", true);
    wind_node = pyGetNode("/filters/wind", true);
    act_node = pyGetNode("/actuators/actuator", true);
#define NUM_ACTUATORS 8
    act_node.setLen("channel", NUM_ACTUATORS, 0.0);
    pilot_node = pyGetNode("/sensors/pilot_inputs", true);
    targets_node = pyGetNode("/autopilot/targets", true);
    route_node = pyGetNode("/task/route", true);
    status_node = pyGetNode("/status", true);
    apm2_node = pyGetNode("/sensors/APM2", true);
    remote_link_node = pyGetNode("/comms/remote_link", true);
}


int UGPacketizer::packetize_gps( uint8_t *buf ) {
    uint8_t *startbuf = buf;

    double time = gps_node.getDouble("timestamp");
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

    double lat = gps_node.getDouble("latitude_deg");
    // *(double *)buf = lat; buf += 8;
    memcpy( buf, &lat, 8 ); buf += 8;

    double lon = gps_node.getDouble("longitude_deg");
    // *(double *)buf = lon; buf += 8;
    memcpy( buf, &lon, 8 ); buf += 8;

    float alt = gps_node.getDouble("altitude_m");
    // *(float *)buf = alt; buf += 4;
    memcpy( buf, &alt, 4 ); buf += 4;

    /* +/- 327.67 mps (732.9 mph), resolution of 0.01 mps */
    int16_t vn = (int16_t)(gps_node.getDouble("vn_ms") * 100);
    *(int16_t *)buf = vn; buf += 2;

    int16_t ve = (int16_t)(gps_node.getDouble("ve_ms") * 100);
    *(int16_t *)buf = ve; buf += 2;

    int16_t vd = (int16_t)(gps_node.getDouble("vd_ms") * 100);
    *(int16_t *)buf = vd; buf += 2;
    
    double date = gps_node.getDouble("unix_time_sec");
    // *(double *)buf = date; buf += 8;
    memcpy( buf, &date, 8 ); buf += 8;

    uint8_t sats = gps_node.getLong("satellites");
    *buf = sats; buf++;

    uint8_t status = 0;
    *buf = status; buf++;

    return buf - startbuf;
}


void UGPacketizer::decode_gps( uint8_t *buf ) {
    double time = *(double *)buf; buf += 8;
    double lat = *(double *)buf; buf += 8;
    double lon = *(double *)buf; buf += 8;
    float alt = *(float *)buf; buf += 4;
    int16_t vn = *(int16_t *)buf; buf += 2;
    int16_t ve = *(int16_t *)buf; buf += 2;
    int16_t vd = *(int16_t *)buf; buf += 2;
    double date = *(double *)buf; buf += 8;
    uint8_t status = *(uint8_t *)buf; buf += 1;

    printf("t = %.2f (%.8f %.8f) a=%.2f  (%.2f %.2f %.2f) %.2f %d\n",
	   time, lat, lon, alt, vn/100.0, ve/100.0, vd/100.0, date,
	   status);
}


int UGPacketizer::packetize_imu( uint8_t *buf ) {
    uint8_t *startbuf = buf;

    double time = imu_node.getDouble("timestamp");
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

    float p = imu_node.getDouble("p_rad_sec");
    // *(float *)buf = p; buf += 4;
    memcpy( buf, &p, 4 ); buf += 4;

    float q = imu_node.getDouble("q_rad_sec");
    // *(float *)buf = q; buf += 4;
    memcpy( buf, &q, 4 ); buf += 4;

    float r = imu_node.getDouble("r_rad_sec");
    // *(float *)buf = r; buf += 4;
    memcpy( buf, &r, 4 ); buf += 4;

    float ax = imu_node.getDouble("ax_mps_sec");
    // *(float *)buf = ax; buf += 4;
    memcpy( buf, &ax, 4 ); buf += 4;

    float ay = imu_node.getDouble("ay_mps_sec");
    // *(float *)buf = ay; buf += 4;
    memcpy( buf, &ay, 4 ); buf += 4;

    float az = imu_node.getDouble("az_mps_sec");
    // *(float *)buf = az; buf += 4;
    memcpy( buf, &az, 4 ); buf += 4;

    float hx = imu_node.getDouble("hx");
    // *(float *)buf = hx; buf += 4;
    memcpy( buf, &hx, 4 ); buf += 4;

    float hy = imu_node.getDouble("hy");
    // *(float *)buf = hy; buf += 4;
    memcpy( buf, &hy, 4 ); buf += 4;

    float hz = imu_node.getDouble("hz");
    // *(float *)buf = hz; buf += 4;
    memcpy( buf, &hz, 4 ); buf += 4;

    int16_t temp = (int16_t)(imu_node.getDouble("temp_C") * 10);
    *(int16_t *)buf = temp; buf += 2;

    uint8_t status = 0;
    *buf = status; buf++;

    return buf - startbuf;
}


void UGPacketizer::decode_imu( uint8_t *buf ) {
    double time = *(double *)buf; buf += 8;
    float p = *(float *)buf; buf += 4;
    float q = *(float *)buf; buf += 4;
    float r = *(float *)buf; buf += 4;
    float ax = *(float *)buf; buf += 4;
    float ay = *(float *)buf; buf += 4;
    float az = *(float *)buf; buf += 4;
    float hx = *(float *)buf; buf += 4;
    float hy = *(float *)buf; buf += 4;
    float hz = *(float *)buf; buf += 4;
    int16_t temp = *(int16_t *)buf; buf += 2;
    uint8_t status = *(uint8_t *)buf; buf += 1;

    printf("t = %.2f (%.3f %.3f %.3f) (%.3f %.3f %.f) (%.3f %.3f %.3f) %.1fC %d\n",
	   time, p, q, r, ax, ay, az, hx, hy, hz, (float)temp/10.0, status );
}


int UGPacketizer::packetize_airdata( uint8_t *buf ) {
    uint8_t *startbuf = buf;

    double time = airdata_node.getDouble("timestamp");
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;
    
    uint16_t mbar = (uint16_t)(airdata_node.getDouble("pressure_mbar") * 10);
    *(uint16_t *)buf = mbar; buf += 2;

    int16_t temp = (int16_t)(airdata_node.getDouble("temp_degC") * 10);
    *(int16_t *)buf = temp; buf += 2;

    // double airspeed_kt = vel_node.getDouble("airspeed_kt");
    double airspeed_kt = vel_node.getDouble("airspeed_smoothed_kt");
    int16_t airspeed = (int16_t)(airspeed_kt * 100);
    *(int16_t *)buf = airspeed; buf += 2;

    double alt_m = pos_pressure_node.getDouble("altitude_smoothed_m");
    // double alt_m = pos_combined_node.getDouble("altitude_true_m");
    // double alt_m = pos_node.getDouble("altitude_agl_ft") * 0.3048;
    float alt = alt_m;
    // *(float *)buf = alt; buf += 4;
    memcpy( buf, &alt, 4 ); buf+= 4;

    float alt_true = pos_combined_node.getDouble("altitude_true_m");
    // *(float *)buf = alt_true; buf += 4;
    memcpy( buf, &alt_true, 4 ); buf+= 4;

    int16_t climb = (int16_t)((vel_node.getDouble("pressure_vertical_speed_fps") * 60) * 10);
    *(int16_t *)buf = climb; buf += 2;

    int16_t empty = (int16_t)(0);
    *(int16_t *)buf = empty; buf += 2;

    uint16_t wind_deg = (uint16_t)(wind_node.getDouble("wind_dir_deg") * 100);
    *(uint16_t *)buf = wind_deg; buf += 2;

    uint8_t wind_kts = (uint8_t)(wind_node.getDouble("wind_speed_kt") * 4);
    *buf = wind_kts; buf += 1;

    uint8_t pitot_scale = (uint8_t)(wind_node.getDouble("pitot_scale_factor")*100);
    *(uint8_t *)buf = pitot_scale; buf += 1;

    uint8_t status = airdata_node.getLong("status");
    *buf = status; buf++;

    return buf - startbuf;
}


void UGPacketizer::decode_airdata( uint8_t *buf ) {
    double time = *(double *)buf; buf += 8;

    uint16_t mbar = *(uint16_t *)buf; buf += 2;
    int16_t temp = *(int16_t *)buf; buf += 2;
    int16_t airspeed = *(int16_t *)buf; buf += 2;
    float alt = *(float *)buf; buf += 4;
    int16_t climb = *(int16_t *)buf; buf += 2;
    int16_t empty = *(int16_t *)buf; buf += 2;
    uint16_t wind_deg = *(uint16_t *)buf; buf += 2;
    uint8_t wind_kts = *(uint8_t *)buf; buf += 1;
    uint8_t status = *(uint8_t *)buf; buf += 1;

    printf("t = %.2f %.1f %.1f %.1f %.1f %.2f %.2f %.1f %.1f %d\n",
	   time, (float)mbar/10, (float)temp/10.0, (float)airspeed/100.0,
	   alt, (float)climb/10.0, (float)empty, (float)wind_deg/100.0,
	   (float)wind_kts/4.0, status );
}


int UGPacketizer::packetize_filter( uint8_t *buf ) {
    uint8_t *startbuf = buf;

    double time = filter_node.getDouble("timestamp");
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

    double lat = filter_node.getDouble("latitude_deg");
    // *(double *)buf = lat; buf += 8;
    memcpy( buf, &lat, 8 ); buf += 8;

    double lon = filter_node.getDouble("longitude_deg");
    // *(double *)buf = lon; buf += 8;
    memcpy( buf, &lon, 8 ); buf += 8;

    float alt = filter_node.getDouble("altitude_m");
    // *(float *)buf = alt; buf += 4;
    memcpy( buf, &alt, 4 ); buf += 4;

    /* +/- 327.67 mps (732.9 mph), resolution of 0.01 mps */
    int16_t vn = (int16_t)(filter_node.getDouble("vn_ms") * 100);
    *(int16_t *)buf = vn; buf += 2;

    int16_t ve = (int16_t)(filter_node.getDouble("ve_ms") * 100);
    *(int16_t *)buf = ve; buf += 2;

    int16_t vd = (int16_t)(filter_node.getDouble("vd_ms") * 100);
    *(int16_t *)buf = vd; buf += 2;

    /* resolution of 0.1 degrees */
    int16_t phi = (int16_t)(filter_node.getDouble("roll_deg") * 10);
    *(int16_t *)buf = phi; buf += 2;

    int16_t theta = (int16_t)(filter_node.getDouble("pitch_deg") * 10);
    *(int16_t *)buf = theta; buf += 2;

    int16_t psi = (int16_t)(filter_node.getDouble("heading_deg") * 10);
    *(int16_t *)buf = psi; buf += 2;

    int8_t seq = (int8_t)remote_link_node.getLong("sequence_num");
    *(int8_t *)buf = seq; buf++;

    uint8_t status = 0;
    *buf = status; buf++;

    return buf - startbuf;
}


void UGPacketizer::decode_filter( uint8_t *buf ) {
    double time = *(double *)buf; buf += 8;
    double lat = *(double *)buf; buf += 8;
    double lon = *(double *)buf; buf += 8;
    float alt = *(float *)buf; buf += 4;
    int16_t vn = *(int16_t *)buf; buf += 2;
    int16_t ve = *(int16_t *)buf; buf += 2;
    int16_t vd = *(int16_t *)buf; buf += 2;
    int16_t phi = *(int16_t *)buf; buf += 2;
    int16_t the = *(int16_t *)buf; buf += 2;
    int16_t psi = *(int16_t *)buf; buf += 2;
    uint8_t seq = *(uint8_t *)buf; buf += 1;
    uint8_t status = *(uint8_t *)buf; buf += 1;

    printf("t = %.2f (%.8f %.8f a=%.2f) (%.2f %.2f %.2f) (%.1f %.1f %.1f) %d %d\n",
	   time, lat, lon, alt,
	   vn/100.0, ve/100.0, vd/100.0,
	   phi/10.0, the/10.0, psi/10.0,
	   seq, status);
}


int UGPacketizer::packetize_actuator( uint8_t *buf ) {
    uint8_t *startbuf = buf;

    double time = act_node.getDouble("timestamp");
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

    int16_t ail = (int16_t)(act_node.getDouble("channel", 0) * 30000);
    *(int16_t *)buf = ail; buf += 2;

    int16_t ele = (int16_t)(act_node.getDouble("channel", 1) * 30000);
    *(int16_t *)buf = ele; buf += 2;

    uint16_t thr = (uint16_t)(act_node.getDouble("channel", 2) * 60000);
    *(uint16_t *)buf = thr; buf += 2;

    int16_t rud = (int16_t)(act_node.getDouble("channel", 3) * 30000);
    *(int16_t *)buf = rud; buf += 2;

    int16_t ch5 = (int16_t)(act_node.getDouble("channel", 4) * 30000);
    *(int16_t *)buf = ch5; buf += 2;

    int16_t ch6 = (int16_t)(act_node.getDouble("channel", 5) * 30000);
    *(int16_t *)buf = ch6; buf += 2;

    int16_t ch7 = (int16_t)(act_node.getDouble("channel", 6) * 30000);
    *(int16_t *)buf = ch7; buf += 2;

    int16_t ch8 = (int16_t)(act_node.getDouble("channel", 7) * 30000);
    *(int16_t *)buf = ch8; buf += 2;

    uint8_t status = 0;
    *buf = status; buf++;

    return buf - startbuf;
}


void UGPacketizer::decode_actuator( uint8_t *buf ) {
    double time = *(double *)buf; buf += 8;
    int16_t ail = *(int16_t *)buf; buf += 2;
    int16_t ele = *(int16_t *)buf; buf += 2;
    uint16_t thr = *(uint16_t *)buf; buf += 2;
    int16_t rud = *(int16_t *)buf; buf += 2;
    int16_t ch5 = *(int16_t *)buf; buf += 2;
    int16_t ch6 = *(int16_t *)buf; buf += 2;
    int16_t ch7 = *(int16_t *)buf; buf += 2;
    int16_t ch8 = *(int16_t *)buf; buf += 2;
    uint8_t status = *(uint8_t *)buf; buf += 1;

    printf("t = %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d\n",
	   time,
	   ail/30000.0, ele/30000.0, thr/60000.0, rud/30000.0,
	   ch5/30000.0, ch6/30000.0, ch7/30000.0, ch8/30000.0,
	   status);
}


int UGPacketizer::packetize_pilot( uint8_t *buf ) {
    uint8_t *startbuf = buf;

    double time = pilot_node.getDouble("timestamp");
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

    int16_t ail = (int16_t)(pilot_node.getDouble("aileron") * 30000);
    *(int16_t *)buf = ail; buf += 2;

    int16_t ele = (int16_t)(pilot_node.getDouble("elevator") * 30000);
    *(int16_t *)buf = ele; buf += 2;

    uint16_t thr = (uint16_t)(pilot_node.getDouble("throttle") * 60000);
    *(uint16_t *)buf = thr; buf += 2;

    int16_t rud = (int16_t)(pilot_node.getDouble("rudder") * 30000);
    *(int16_t *)buf = rud; buf += 2;

    int16_t ch5 = (int16_t)(pilot_node.getDouble("manual") * 30000);
    *(int16_t *)buf = ch5; buf += 2;

    int16_t ch6 = (int16_t)(pilot_node.getDouble("channel", 5) * 30000);
    *(int16_t *)buf = ch6; buf += 2;

    int16_t ch7 = (int16_t)(pilot_node.getDouble("channel", 6) * 30000);
    *(int16_t *)buf = ch7; buf += 2;

    int16_t ch8 = (int16_t)(pilot_node.getDouble("channel", 7) * 30000);
    *(int16_t *)buf = ch8; buf += 2;

    uint8_t status = 0;
    *buf = status; buf++;

    return buf - startbuf;
}


void UGPacketizer::decode_pilot( uint8_t *buf ) {
    double time = *(double *)buf; buf += 8;
    int16_t ail = *(int16_t *)buf; buf += 2;
    int16_t ele = *(int16_t *)buf; buf += 2;
    uint16_t thr = *(uint16_t *)buf; buf += 2;
    int16_t rud = *(int16_t *)buf; buf += 2;
    int16_t ch5 = *(int16_t *)buf; buf += 2;
    int16_t ch6 = *(int16_t *)buf; buf += 2;
    int16_t ch7 = *(int16_t *)buf; buf += 2;
    int16_t ch8 = *(int16_t *)buf; buf += 2;
    uint8_t status = *(uint8_t *)buf; buf += 1;

    printf("t = %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d\n",
	   time,
	   ail/30000.0, ele/30000.0, thr/60000.0, rud/30000.0,
	   ch5/30000.0, ch6/30000.0, ch7/30000.0, ch8/30000.0,
	   status);
}


int UGPacketizer::packetize_ap( uint8_t *buf, uint8_t route_size,
				SGWayPoint *wp, int index )
{
    uint8_t *startbuf = buf;

    double time = get_Time();
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

    int16_t hdg = (int16_t)(targets_node.getDouble("groundtrack_deg") * 10.0);
    *(int16_t *)buf = hdg; buf += 2;

    int16_t roll = (int16_t)(targets_node.getDouble("roll_deg") * 10.0);
    *(int16_t *)buf = roll; buf += 2;

    // compute target AP msl as:
    //   ground_alt(pressure) + altitude_agl(press) - error(press)
    float target_agl_ft = targets_node.getDouble("altitude_agl_ft");
    float ground_m = pos_pressure_node.getDouble("altitude_ground_m");
    float error_m = pos_pressure_node.getDouble("pressure_error_m");
    float alt_msl_ft = (ground_m + error_m) * SG_METER_TO_FEET + target_agl_ft;
    *(uint16_t *)buf = (uint16_t)alt_msl_ft; buf += 2;

    int16_t climb = (int16_t)(targets_node.getDouble("climb_rate_fps") * 10.0);
    *(int16_t *)buf = climb; buf += 2;

    int16_t pitch = (int16_t)(targets_node.getDouble("pitch_deg") * 10.0);
    *(int16_t *)buf = pitch; buf += 2;

    int16_t theta_dot = (int16_t)(targets_node.getDouble("the_dot") * 1000.0);
    *(int16_t *)buf = theta_dot; buf += 2;

    int16_t speed = (int16_t)(targets_node.getDouble("airspeed_kt") * 10.0);
    *(int16_t *)buf = speed; buf += 2;

    int target_wpt_index = route_node.getLong("target_waypoint_idx");
    if ( target_wpt_index >= route_size ) {
	target_wpt_index = 0;
    }
    uint16_t waypoint = (uint16_t)target_wpt_index;
    *(uint16_t *)buf = waypoint; buf += 2;

    // *(double *)buf = wp.get_target_lon(); buf += 8;
    double tlon = wp->get_target_lon();
    memcpy( buf, &tlon, 8 ); buf += 8;
    // *(double *)buf = wp.get_target_lat(); buf += 8;
    double tlat = wp->get_target_lat();
    memcpy( buf, &tlat, 8 ); buf += 8;
    *(uint16_t *)buf = index; buf += 2;
    *(uint16_t *)buf = route_size; buf += 2;

    int8_t seq = (int8_t)remote_link_node.getLong("sequence_num");
    *(int8_t *)buf = seq; buf++;

    return buf - startbuf;
}


void UGPacketizer::decode_ap( uint8_t *buf ) {
    double time = *(double *)buf; buf += 8;
    int16_t ap_hdg = *(int16_t *)buf; buf += 2;
    int16_t ap_roll = *(int16_t *)buf; buf += 2;
    uint16_t ap_alt_agl = *(uint16_t *)buf; buf += 2;
    uint16_t ap_alt_msl = *(uint16_t *)buf; buf += 2;
    int16_t ap_climb = *(int16_t *)buf; buf += 2;
    int16_t ap_pitch = *(int16_t *)buf; buf += 2;
    int16_t ap_theta_dot = *(int16_t *)buf; buf += 2;
    int16_t ap_speed = *(int16_t *)buf; buf += 2;
    uint16_t ap_waypoint = *(uint16_t *)buf; buf += 2;
    double lon = *(double *)buf; buf += 8;
    double lat = *(double *)buf; buf += 8;
    uint16_t wp_index = *(uint16_t *)buf; buf += 2;
    uint16_t route_size = *(uint16_t *)buf; buf += 2;

    printf("t = %.2f %.1f %.1f %d %d %d %.1f %.3f %.1f %d %.10f %.10f %d %d\n",
	   time,
	   ap_hdg/10.0, ap_roll/10.0, ap_alt_agl, ap_alt_msl, ap_climb,
	   ap_pitch/10.0, ap_theta_dot/1000.0, ap_speed/10.0, ap_waypoint,
	   lon, lat, wp_index,
	   route_size);
}

int UGPacketizer::packetize_health( uint8_t *buf )
{
    uint8_t *startbuf = buf;

    double time = get_Time();
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

    uint16_t loadavg = (uint16_t)(status_node.getDouble("system_load_avg") * 100.0);
    *(uint16_t *)buf = loadavg; buf += 2;

    uint16_t vcc = (uint16_t)(apm2_node.getDouble("board_vcc") * 1000.0);
    *(uint16_t *)buf = vcc; buf += 2;

    uint16_t volt = (uint16_t)(apm2_node.getDouble("extern_volt") * 1000.0);
    *(uint16_t *)buf = volt; buf += 2;

    uint16_t cell_volt = (uint16_t)(apm2_node.getDouble("extern_cell_volt") * 1000.0);
    *(uint16_t *)buf = cell_volt; buf += 2;

    uint16_t amp = (uint16_t)(apm2_node.getDouble("extern_amps") * 1000.0);
    *(uint16_t *)buf = amp; buf += 2;

    uint16_t mah = (uint16_t)(apm2_node.getDouble("extern_current_mah"));
    *(uint16_t *)buf = mah; buf += 2;

    return buf - startbuf;
}


void UGPacketizer::decode_health( uint8_t *buf ) {
    double time = *(double *)buf; buf += 8;
    uint16_t loadavg = *(uint16_t *)buf; buf +=- 2;
    uint16_t avionics_vcc = *(uint16_t *)buf; buf += 2;
    uint16_t volt = *(uint16_t *)buf; buf += 2;
    uint16_t amp = *(uint16_t *)buf; buf += 2;
    uint16_t mah = *(uint16_t *)buf; buf += 2;

    printf("t = %.2f %.3f %.2f %.2f %.1f %d \n",
	   time,
	   loadavg/10.0, avionics_vcc/1000.0, volt/1000.0, amp/1000.0, mah );
}


int UGPacketizer::packetize_payload( uint8_t *buf )
{
    uint8_t *startbuf = buf;

    double time = get_Time();
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

    uint16_t trigger_num = (uint16_t)(payload_node.getLong("trigger_num"));
    *(uint16_t *)buf = trigger_num; buf += 2;

    return buf - startbuf;
}


void UGPacketizer::decode_payload( uint8_t *buf ) {
    double time = *(double *)buf; buf += 8;
    uint16_t trigger_num = *(uint16_t *)buf; buf +=- 2;

    printf("t = %.2f %d\n",
	   time,
	   trigger_num );
}


string UGPacketizer::get_fcs_nav_string() {
    static int max_buf = 256;
    char buf[max_buf];

    double filter_hdg = (SGD_PI * 0.5 - atan2(filter_node.getDouble("vn_ms"), filter_node.getDouble("ve_ms"))) * SG_RADIANS_TO_DEGREES;
    snprintf(buf, max_buf, "%.2f,%.1f,%.1f,%.1f,%.1f,%.2f",
	     imu_node.getDouble("timestamp"),
	     targets_node.getDouble("groundtrack_deg"),
	     targets_node.getDouble("roll_deg"),
	     filter_hdg,
	     filter_node.getDouble("roll_deg"),
	     act_node.getDouble("channel", 0));

    return buf;
}

string UGPacketizer::get_fcs_speed_string() {
    static int max_buf = 256;
    char buf[max_buf];

    snprintf(buf, max_buf, "%.2f,%.1f,%.1f,%.1f,%.1f,%.2f",
	     imu_node.getDouble("timestamp"),
	     targets_node.getDouble("airspeed_kt"),
	     targets_node.getDouble("pitch_deg"),
	     vel_node.getDouble("airspeed_smoothed_kt"),
	     filter_node.getDouble("pitch_deg"),
	     act_node.getDouble("channel", 1));

    return buf;
}

string UGPacketizer::get_fcs_altitude_string() {
    static int max_buf = 256;
    char buf[max_buf];

    snprintf(buf, max_buf, "%.2f,%.1f,%.1f,%.2f",
	     imu_node.getDouble("timestamp"),
	     targets_node.getDouble("altitude_agl_ft"),
	     pos_node.getDouble("altitude_agl_ft"),
	     act_node.getDouble("channel", 2) );

    return buf;
}

bool UGPacketizer::decode_fcs_update(vector <string> tokens) {
    if ( tokens.size() > 0 && tokens[0] == "fcs-update" ) {
	// remove initial keyword if needed
	tokens.erase(tokens.begin());
    }

    if ( tokens.size() == 9 ) {
	int i = atoi(tokens[0].c_str());
	ostringstream str;
	str << "/config/fcs/autopilot/pid-controller" << '[' << i << ']';
	string ename = str.str();
	pyPropertyNode pid = pyGetNode(ename);
	if ( pid.isNull() ) {
	    return false;
	}

	pyPropertyNode config = pid.getChild("config");
	if ( config.isNull() ) {
	    return false;
	}
	
	config.setDouble( "Kp", atof(tokens[1].c_str()) );
	config.setDouble( "beta", atof(tokens[2].c_str()) );
	config.setDouble( "alpha", atof(tokens[3].c_str()) );
	config.setDouble( "gamma", atof(tokens[4].c_str()) );
	config.setDouble( "Ti", atof(tokens[5].c_str()) );
	config.setDouble( "Td", atof(tokens[6].c_str()) );
	config.setDouble( "u_min", atof(tokens[7].c_str()) );
	config.setDouble( "u_max", atof(tokens[8].c_str()) );

	return true;
    } else if ( tokens.size() == 5 ) {
	int i = atoi(tokens[0].c_str());
	ostringstream str;
	str << "/config/fcs/autopilot/pi-simple-controller" << '[' << i << ']';
	string ename = str.str();
	pyPropertyNode pid = pyGetNode(ename);
	if ( pid.isNull() ) {
	    return false;
	}

	pyPropertyNode config = pid.getChild("config");
	if ( config.isNull() ) {
	    return false;
	}

	config.setDouble( "Kp", atof(tokens[1].c_str()) );
	config.setDouble( "Ki", atof(tokens[2].c_str()) );
	config.setDouble( "u_min", atof(tokens[3].c_str()) );
	config.setDouble( "u_max", atof(tokens[4].c_str()) );

	return true;
     } else {
	return false;
    }
}

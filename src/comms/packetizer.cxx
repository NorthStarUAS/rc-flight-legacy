#include <stdint.h>

#include <stdio.h>

#include "control/route_mgr.hxx"
#include "include/globaldefs.h"
#include "util/timing.h"

#include "packetizer.hxx"


// initialize gps property nodes 
void UGPacketizer::bind_gps_nodes() {
    gps_timestamp_node = fgGetNode("/sensors/gps/time-stamp", true);
    gps_lat_node = fgGetNode("/sensors/gps/latitude-deg", true);
    gps_lon_node = fgGetNode("/sensors/gps/longitude-deg", true);
    gps_alt_node = fgGetNode("/sensors/gps/altitude-m", true);
    gps_ve_node = fgGetNode("/sensors/gps/ve-ms", true);
    gps_vn_node = fgGetNode("/sensors/gps/vn-ms", true);
    gps_vd_node = fgGetNode("/sensors/gps/vd-ms", true);
    gps_unix_sec_node = fgGetNode("/sensors/gps/unix-time-sec", true);
    gps_satellites_node = fgGetNode("/sensors/gps/satellites", true);
    gps_status_node = NULL;
}

// initialize imu property nodes 
void UGPacketizer::bind_imu_nodes() {
    imu_timestamp_node = fgGetNode("/sensors/imu/time-stamp", true);
    imu_p_node = fgGetNode("/sensors/imu/p-rad_sec", true);
    imu_q_node = fgGetNode("/sensors/imu/q-rad_sec", true);
    imu_r_node = fgGetNode("/sensors/imu/r-rad_sec", true);
    imu_ax_node = fgGetNode("/sensors/imu/ax-mps_sec", true);
    imu_ay_node = fgGetNode("/sensors/imu/ay-mps_sec", true);
    imu_az_node = fgGetNode("/sensors/imu/az-mps_sec", true);
    imu_hx_node = fgGetNode("/sensors/imu/hx", true);
    imu_hy_node = fgGetNode("/sensors/imu/hy", true);
    imu_hz_node = fgGetNode("/sensors/imu/hz", true);
    imu_status_node = fgGetNode("/sensors/imu/status", true);
}

// initialize air data property nodes 
void UGPacketizer::bind_airdata_nodes() {
    airdata_timestamp_node = fgGetNode("/sensors/air-data/time-stamp", true);
    // airdata_altitude_node = fgGetNode("/sensors/air-data/altitude-m", true);
    airdata_altitude_node = fgGetNode("/position/altitude-pressure-m", true);
    // airdata_airspeed_node = fgGetNode("/sensors/air-data/airspeed-kt", true);
    airdata_airspeed_node = fgGetNode("/velocity/airspeed-kt", true);
    airdata_climb_fps_node
	= fgGetNode("/velocity/pressure-vertical-speed-fps",true);
    airdata_accel_ktps_node = fgGetNode("/acceleration/airspeed-ktps",true);
    airdata_status_node = fgGetNode("/sensors/air-data/status", true);
}

// initialize filter property nodes
void UGPacketizer::bind_filter_nodes() {
    filter_timestamp_node = fgGetNode("/filters/filter/time-stamp", true);
    filter_theta_node = fgGetNode("/filters/filter/pitch-deg", true);
    filter_phi_node = fgGetNode("/filters/filter/roll-deg", true);
    filter_psi_node = fgGetNode("/filters/filter/heading-deg", true);
    filter_lat_node = fgGetNode("/filters/filter/latitude-deg", true);
    filter_lon_node = fgGetNode("/filters/filter/longitude-deg", true);
    filter_alt_node = fgGetNode("/filters/filter/altitude-m", true);
    filter_vn_node = fgGetNode("/filters/filter/vn-ms", true);
    filter_ve_node = fgGetNode("/filters/filter/ve-ms", true);
    filter_vd_node = fgGetNode("/filters/filter/vd-ms", true);
    filter_status_node = fgGetNode("/filters/filter/status", true);
}

// initialize actuator property nodes
void UGPacketizer::bind_actuator_nodes() {
    act_timestamp_node = fgGetNode("/actuators/actuator/time-stamp", true);
    act_aileron_node = fgGetNode("/actuators/actuator/channel", 0, true);
    act_elevator_node = fgGetNode("/actuators/actuator/channel", 1, true);
    act_throttle_node = fgGetNode("/actuators/actuator/channel", 2, true);
    act_rudder_node = fgGetNode("/actuators/actuator/channel", 3, true);
    act_channel5_node = fgGetNode("/actuators/actuator/channel", 4, true);
    act_channel6_node = fgGetNode("/actuators/actuator/channel", 5, true);
    act_channel7_node = fgGetNode("/actuators/actuator/channel", 6, true);
    act_channel8_node = fgGetNode("/actuators/actuator/channel", 7, true);
    act_status_node = fgGetNode("/actuators/actuator/status", true);
}


// initialize pilot input property nodes
void UGPacketizer::bind_pilot_nodes() {
    pilot_timestamp_node = fgGetNode("/actuators/pilot/time-stamp", true);
    pilot_aileron_node = fgGetNode("/actuators/pilot/channel", 0, true);
    pilot_elevator_node = fgGetNode("/actuators/pilot/channel", 1, true);
    pilot_throttle_node = fgGetNode("/actuators/pilot/channel", 2, true);
    pilot_rudder_node = fgGetNode("/actuators/pilot/channel", 3, true);
    pilot_channel5_node = fgGetNode("/actuators/pilot/channel", 4, true);
    pilot_channel6_node = fgGetNode("/actuators/pilot/channel", 5, true);
    pilot_channel7_node = fgGetNode("/actuators/pilot/channel", 6, true);
    pilot_channel8_node = fgGetNode("/actuators/pilot/channel", 7, true);
    pilot_status_node = fgGetNode("/actuators/pilot/status", true);
}


// initialize autopilot status property nodes
void UGPacketizer::bind_ap_nodes() {
    filter_ground_alt_m_node
	= fgGetNode("/position/ground-altitude-filter-m", true);
    ap_hdg = fgGetNode( "/autopilot/settings/true-heading-deg", true );
    ap_roll = fgGetNode("/autopilot/internal/target-roll-deg", true);
    ap_altitude_agl = fgGetNode( "/autopilot/settings/target-agl-ft", true );
    ap_altitude_msl = fgGetNode( "/autopilot/settings/target-msl-ft", true );
    ap_climb = fgGetNode("/autopilot/internal/target-climb-rate-fps", true);
    ap_pitch = fgGetNode( "/autopilot/settings/target-pitch-deg", true );
    ap_speed = fgGetNode( "/autopilot/settings/target-speed-kt", true );
    ap_waypoint = fgGetNode( "/autopilot/route-mgr/target-waypoint-idx", true );
}


// initialize system health property nodes
void UGPacketizer::bind_health_nodes() {
    system_load_avg = fgGetNode("/status/system-load-avg", true);
}


UGPacketizer::UGPacketizer() {
    bind_gps_nodes();
    bind_imu_nodes();
    bind_airdata_nodes();
    bind_filter_nodes();
    bind_actuator_nodes();
    bind_pilot_nodes();
    bind_ap_nodes();
    bind_health_nodes();

    // command sequence node
    console_seq_num = fgGetNode("/status/console-link-sequence-num", true);
}


int UGPacketizer::packetize_gps( uint8_t *buf ) {
    uint8_t *startbuf = buf;

    double time = gps_timestamp_node->getDoubleValue();
    *(double *)buf = time; buf += 8;

    double lat = gps_lat_node->getDoubleValue();
    *(double *)buf = lat; buf += 8;

    double lon = gps_lon_node->getDoubleValue();
    *(double *)buf = lon; buf += 8;

    float alt = gps_alt_node->getFloatValue();
    *(float *)buf = alt; buf += 4;

    /* +/- 327.67 mps (732.9 mph), resolution of 0.01 mps */
    int16_t vn = (int16_t)(gps_vn_node->getDoubleValue() * 100);
    *(int16_t *)buf = vn; buf += 2;

    int16_t ve = (int16_t)(gps_ve_node->getDoubleValue() * 100);
    *(int16_t *)buf = ve; buf += 2;

    int16_t vd = (int16_t)(gps_vd_node->getDoubleValue() * 100);
    *(int16_t *)buf = vd; buf += 2;
    
    double date = gps_unix_sec_node->getDoubleValue();
    *(double *)buf = date; buf += 8;

    uint8_t sats = gps_satellites_node->getIntValue();
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

    double time = imu_timestamp_node->getDoubleValue();
    *(double *)buf = time; buf += 8;

    float p = imu_p_node->getFloatValue();
    *(float *)buf = p; buf += 4;

    float q = imu_q_node->getFloatValue();
    *(float *)buf = q; buf += 4;

    float r = imu_r_node->getFloatValue();
    *(float *)buf = r; buf += 4;

    float ax = imu_ax_node->getFloatValue();
    *(float *)buf = ax; buf += 4;

    float ay = imu_ay_node->getFloatValue();
    *(float *)buf = ay; buf += 4;

    float az = imu_az_node->getFloatValue();
    *(float *)buf = az; buf += 4;

    float hx = imu_hx_node->getFloatValue();
    *(float *)buf = hx; buf += 4;

    float hy = imu_hy_node->getFloatValue();
    *(float *)buf = hy; buf += 4;

    float hz = imu_hz_node->getFloatValue();
    *(float *)buf = hz; buf += 4;

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
    uint8_t status = *(uint8_t *)buf; buf += 1;

    printf("t = %.2f (%.3f %.3f %.3f) (%.3f %.3f %.f) (%.3f %.3f %.3f) %d\n",
	   time, p, q, r, ax, ay, az, hx, hy, hz, status );
}


int UGPacketizer::packetize_airdata( uint8_t *buf ) {
    uint8_t *startbuf = buf;

    double time = airdata_timestamp_node->getDoubleValue();
    *(double *)buf = time; buf += 8;

    int16_t airspeed = (int16_t)(airdata_airspeed_node->getFloatValue() * 100);
    *(int16_t *)buf = airspeed; buf += 2;

    float alt = airdata_altitude_node->getFloatValue();
    *(float *)buf = alt; buf += 4;

    int16_t climb = (int16_t)((airdata_climb_fps_node->getFloatValue() * 60) * 10);
    *(float *)buf = climb; buf += 2;

    int16_t accel = (int16_t)(airdata_accel_ktps_node->getFloatValue() * 100);
    *(float *)buf = accel; buf += 2;

    uint8_t status = airdata_status_node->getIntValue();
    *buf = status; buf++;

    return buf - startbuf;
}


void UGPacketizer::decode_airdata( uint8_t *buf ) {
    double time = *(double *)buf; buf += 8;
    int16_t airspeed = *(int16_t *)buf; buf += 2;
    float alt = *(float *)buf; buf += 4;
    int16_t climb = *(int16_t *)buf; buf += 2;
    int16_t accel = *(int16_t *)buf; buf += 2;
    uint8_t status = *(uint8_t *)buf; buf += 1;

    printf("t = %.2f %.1f %.1f %.2f %.2f %d\n",
	   time, (float)airspeed/100, alt, (float)climb/10, (float)accel/100,
	   status );
}


int UGPacketizer::packetize_filter( uint8_t *buf ) {
    uint8_t *startbuf = buf;

    double time = filter_timestamp_node->getDoubleValue();
    *(double *)buf = time; buf += 8;

    double lat = filter_lat_node->getDoubleValue();
    *(double *)buf = lat; buf += 8;

    double lon = filter_lon_node->getDoubleValue();
    *(double *)buf = lon; buf += 8;

    float alt = filter_alt_node->getFloatValue();
    *(float *)buf = alt; buf += 4;

    /* +/- 327.67 mps (732.9 mph), resolution of 0.01 mps */
    int16_t vn = (int16_t)(filter_vn_node->getDoubleValue() * 100);
    *(int16_t *)buf = vn; buf += 2;

    int16_t ve = (int16_t)(filter_ve_node->getDoubleValue() * 100);
    *(int16_t *)buf = ve; buf += 2;

    int16_t vd = (int16_t)(filter_vd_node->getDoubleValue() * 100);
    *(int16_t *)buf = vd; buf += 2;

    /* resolution of 0.1 degrees */
    int16_t phi = (int16_t)(filter_phi_node->getDoubleValue() * 10);
    *(int16_t *)buf = phi; buf += 2;

    int16_t theta = (int16_t)(filter_theta_node->getDoubleValue() * 10);
    *(int16_t *)buf = theta; buf += 2;

    int16_t psi = (int16_t)(filter_psi_node->getDoubleValue() * 10);
    *(int16_t *)buf = psi; buf += 2;

    int8_t seq = (int8_t)console_seq_num->getIntValue();
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

    double time = act_timestamp_node->getDoubleValue();
    *(double *)buf = time; buf += 8;

    int16_t ail = (int16_t)(act_aileron_node->getDoubleValue() * 30000);
    *(int16_t *)buf = ail; buf += 2;

    int16_t ele = (int16_t)(act_elevator_node->getDoubleValue() * 30000);
    *(int16_t *)buf = ele; buf += 2;

    uint16_t thr = (uint16_t)(act_throttle_node->getDoubleValue() * 60000);
    *(uint16_t *)buf = thr; buf += 2;

    int16_t rud = (int16_t)(act_rudder_node->getDoubleValue() * 30000);
    *(int16_t *)buf = rud; buf += 2;

    int16_t ch5 = (int16_t)(act_channel5_node->getDoubleValue() * 30000);
    *(int16_t *)buf = ch5; buf += 2;

    int16_t ch6 = (int16_t)(act_channel6_node->getDoubleValue() * 30000);
    *(int16_t *)buf = ch6; buf += 2;

    int16_t ch7 = (int16_t)(act_channel7_node->getDoubleValue() * 30000);
    *(int16_t *)buf = ch7; buf += 2;

    int16_t ch8 = (int16_t)(act_channel8_node->getDoubleValue() * 30000);
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

    double time = pilot_timestamp_node->getDoubleValue();
    *(double *)buf = time; buf += 8;

    int16_t ail = (int16_t)(pilot_aileron_node->getDoubleValue() * 30000);
    *(int16_t *)buf = ail; buf += 2;

    int16_t ele = (int16_t)(pilot_elevator_node->getDoubleValue() * 30000);
    *(int16_t *)buf = ele; buf += 2;

    uint16_t thr = (uint16_t)(pilot_throttle_node->getDoubleValue() * 60000);
    *(uint16_t *)buf = thr; buf += 2;

    int16_t rud = (int16_t)(pilot_rudder_node->getDoubleValue() * 30000);
    *(int16_t *)buf = rud; buf += 2;

    int16_t ch5 = (int16_t)(pilot_channel5_node->getDoubleValue() * 30000);
    *(int16_t *)buf = ch5; buf += 2;

    int16_t ch6 = (int16_t)(pilot_channel6_node->getDoubleValue() * 30000);
    *(int16_t *)buf = ch6; buf += 2;

    int16_t ch7 = (int16_t)(pilot_channel7_node->getDoubleValue() * 30000);
    *(int16_t *)buf = ch7; buf += 2;

    int16_t ch8 = (int16_t)(pilot_channel8_node->getDoubleValue() * 30000);
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


int UGPacketizer::packetize_ap( uint8_t *buf, SGWayPoint *wp, int index ) {
    uint8_t *startbuf = buf;

    double time = get_Time();
    *(double *)buf = time; buf += 8;

    int16_t hdg = (int16_t)(ap_hdg->getFloatValue() * 10.0);
    *(int16_t *)buf = hdg; buf += 2;

    int16_t roll = (int16_t)(ap_roll->getFloatValue() * 10.0);
    *(int16_t *)buf = roll; buf += 2;

    float alt_agl_ft = ap_altitude_agl->getFloatValue();
    float ground_m = filter_ground_alt_m_node->getFloatValue();
    float alt_msl_ft = ground_m * SG_METER_TO_FEET + alt_agl_ft;
    *(uint16_t *)buf = (uint16_t)alt_msl_ft; buf += 2;

    int16_t climb = (int16_t)(ap_climb->getFloatValue() * 10.0);
    *(int16_t *)buf = climb; buf += 2;

    int16_t pitch = (int16_t)(ap_pitch->getFloatValue() * 10.0);
    *(int16_t *)buf = pitch; buf += 2;

    int16_t speed = (int16_t)(ap_speed->getFloatValue() * 10.0);
    *(int16_t *)buf = speed; buf += 2;

    uint16_t waypoint = (uint16_t)ap_waypoint->getIntValue();
    *(uint16_t *)buf = waypoint; buf += 2;

    *(double *)buf = wp->get_target_lon(); buf += 8;
    *(double *)buf = wp->get_target_lat(); buf += 8;
    *(uint16_t *)buf = index; buf += 2;
    *(uint16_t *)buf = route_mgr.size(); buf += 2;

    int8_t seq = (int8_t)console_seq_num->getIntValue();
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
    int16_t ap_speed = *(int16_t *)buf; buf += 2;
    uint16_t ap_waypoint = *(uint16_t *)buf; buf += 2;
    double lon = *(double *)buf; buf += 8;
    double lat = *(double *)buf; buf += 8;
    uint16_t wp_index = *(uint16_t *)buf; buf += 2;
    uint16_t route_size = *(uint16_t *)buf; buf += 2;

    printf("t = %.2f %.1f %.1f %d %d %d %.1f %.1f %d %.10f %.10f %d %d\n",
	   time,
	   ap_hdg/10.0, ap_roll/10.0, ap_alt_agl, ap_alt_msl, ap_climb,
	   ap_pitch/10.0, ap_speed/10.0, ap_waypoint, lon, lat, wp_index,
	   route_size);
}

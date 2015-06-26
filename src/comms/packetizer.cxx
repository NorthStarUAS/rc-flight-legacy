#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// #include "control/route_mgr.hxx"
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
    imu_temp_node = fgGetNode("/sensors/imu/temp_C", true);
    imu_status_node = fgGetNode("/sensors/imu/status", true);
}

// initialize air data property nodes 
void UGPacketizer::bind_airdata_nodes() {
    airdata_timestamp_node = fgGetNode("/sensors/airdata/time-stamp", true);
    airdata_pressure_node = fgGetNode("/sensors/airdata/pressure-mbar", true);
    airdata_temperature_node = fgGetNode("/sensors/airdata/temp-degC", true);

    // select one of the following pressure altitude sources
    airdata_altitude_node = fgGetNode("/position/pressure/altitude-smoothed-m", true);
    airdata_altitude_true_node = fgGetNode("/position/combined/altitude-true-m", true);
    official_altitude_agl_node = fgGetNode("/position/altitude-agl-ft", true);

    // select one of the following airspeed sources
    // airdata_airspeed_node = fgGetNode("/sensors/airdata/airspeed-kt", true);
    // airdata_airspeed_node = fgGetNode("/velocity/airspeed-kt", true);
    airdata_airspeed_node = fgGetNode("/velocity/airspeed-smoothed-kt", true);

    // select one of the following climb rate sources
    // airdata_climb_fps_node
    //     = fgGetNode("/sensors/airdata/vertical-speed-fps",true);
    airdata_climb_fps_node
	= fgGetNode("/velocity/pressure-vertical-speed-fps",true);

    airdata_wind_dir_node = fgGetNode("/filters/wind-est/wind-dir-deg", true);
    airdata_wind_speed_node = fgGetNode("/filters/wind-est/wind-speed-kt", true);
    airdata_pitot_scale_node = fgGetNode("/filters/wind-est/pitot-scale-factor", true);
    airdata_status_node = fgGetNode("/sensors/airdata/status", true);
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
    pilot_timestamp_node = fgGetNode("/sensors/pilot/time-stamp", true);
    pilot_aileron_node = fgGetNode("/sensors/pilot/aileron", true);
    pilot_elevator_node = fgGetNode("/sensors/pilot/elevator", true);
    pilot_throttle_node = fgGetNode("/sensors/pilot/throttle", true);
    pilot_rudder_node = fgGetNode("/sensors/pilot/rudder", true);
    pilot_channel5_node = fgGetNode("/sensors/pilot/manual", true);
    pilot_channel6_node = fgGetNode("/sensors/pilot/channel", 5, true);
    pilot_channel7_node = fgGetNode("/sensors/pilot/channel", 6, true);
    pilot_channel8_node = fgGetNode("/sensors/pilot/channel", 7, true);
    pilot_status_node = fgGetNode("/sensors/pilot/status", true);
}


// initialize autopilot status property nodes
void UGPacketizer::bind_ap_nodes() {
    pressure_ground_alt_node
	= fgGetNode("/position/pressure/altitude-ground-m", true);
    pressure_error_node
	= fgGetNode("/position/pressure/pressure-error-m", true);
    ap_hdg = fgGetNode( "/autopilot/settings/target-groundtrack-deg", true );
    ap_roll = fgGetNode("/autopilot/settings/target-roll-deg", true);
    ap_altitude_agl = fgGetNode( "/autopilot/settings/target-agl-ft", true );
    ap_altitude_msl = fgGetNode( "/autopilot/settings/target-msl-ft", true );
    ap_climb = fgGetNode("/autopilot/internal/target-climb-rate-fps", true);
    ap_pitch = fgGetNode( "/autopilot/settings/target-pitch-deg", true );
    ap_theta_dot = fgGetNode( "/autopilot/settings/target-the-dot", true );
    ap_speed = fgGetNode( "/autopilot/settings/target-speed-kt", true );
    ap_waypoint = fgGetNode( "/task/route/target-waypoint-idx", true );
}


// initialize system health property nodes
void UGPacketizer::bind_health_nodes() {
    system_loadavg_node = fgGetNode("/status/system-load-avg", true);
    avionics_vcc_node = fgGetNode("/sensors/APM2/board-vcc", true);
    extern_volt_node = fgGetNode("/sensors/APM2/extern-volt", true);
    extern_cell_volt_node = fgGetNode("/sensors/APM2/extern-cell-volt", true);
    extern_amp_node = fgGetNode("/sensors/APM2/extern-amps", true);
    extern_mah_node = fgGetNode("/sensors/APM2/extern-current-mah", true);
}


// initialize payload status property nodes
void UGPacketizer::bind_payload_nodes() {
    payload_trigger_num_node = fgGetNode("/payload/camera/trigger-num", true);
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
    bind_payload_nodes();

    // command sequence node
    link_seq_num = fgGetNode("/comms/remote-link/sequence-num", true);
}


int UGPacketizer::packetize_gps( uint8_t *buf ) {
    uint8_t *startbuf = buf;

    double time = gps_timestamp_node->getDoubleValue();
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

    double lat = gps_lat_node->getDoubleValue();
    // *(double *)buf = lat; buf += 8;
    memcpy( buf, &lat, 8 ); buf += 8;

    double lon = gps_lon_node->getDoubleValue();
    // *(double *)buf = lon; buf += 8;
    memcpy( buf, &lon, 8 ); buf += 8;

    float alt = gps_alt_node->getFloatValue();
    // *(float *)buf = alt; buf += 4;
    memcpy( buf, &alt, 4 ); buf += 4;

    /* +/- 327.67 mps (732.9 mph), resolution of 0.01 mps */
    int16_t vn = (int16_t)(gps_vn_node->getDoubleValue() * 100);
    *(int16_t *)buf = vn; buf += 2;

    int16_t ve = (int16_t)(gps_ve_node->getDoubleValue() * 100);
    *(int16_t *)buf = ve; buf += 2;

    int16_t vd = (int16_t)(gps_vd_node->getDoubleValue() * 100);
    *(int16_t *)buf = vd; buf += 2;
    
    double date = gps_unix_sec_node->getDoubleValue();
    // *(double *)buf = date; buf += 8;
    memcpy( buf, &date, 8 ); buf += 8;

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
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

    float p = imu_p_node->getFloatValue();
    // *(float *)buf = p; buf += 4;
    memcpy( buf, &p, 4 ); buf += 4;

    float q = imu_q_node->getFloatValue();
    // *(float *)buf = q; buf += 4;
    memcpy( buf, &q, 4 ); buf += 4;

    float r = imu_r_node->getFloatValue();
    // *(float *)buf = r; buf += 4;
    memcpy( buf, &r, 4 ); buf += 4;

    float ax = imu_ax_node->getFloatValue();
    // *(float *)buf = ax; buf += 4;
    memcpy( buf, &ax, 4 ); buf += 4;

    float ay = imu_ay_node->getFloatValue();
    // *(float *)buf = ay; buf += 4;
    memcpy( buf, &ay, 4 ); buf += 4;

    float az = imu_az_node->getFloatValue();
    // *(float *)buf = az; buf += 4;
    memcpy( buf, &az, 4 ); buf += 4;

    float hx = imu_hx_node->getFloatValue();
    // *(float *)buf = hx; buf += 4;
    memcpy( buf, &hx, 4 ); buf += 4;

    float hy = imu_hy_node->getFloatValue();
    // *(float *)buf = hy; buf += 4;
    memcpy( buf, &hy, 4 ); buf += 4;

    float hz = imu_hz_node->getFloatValue();
    // *(float *)buf = hz; buf += 4;
    memcpy( buf, &hz, 4 ); buf += 4;

    int16_t temp = (int16_t)(imu_temp_node->getFloatValue() * 10);
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

    double time = airdata_timestamp_node->getDoubleValue();
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;
    
    uint16_t mbar = (uint16_t)(airdata_pressure_node->getFloatValue() * 10);
    *(uint16_t *)buf = mbar; buf += 2;

    int16_t temp = (int16_t)(airdata_temperature_node->getFloatValue() * 10);
    *(int16_t *)buf = temp; buf += 2;

    int16_t airspeed = (int16_t)(airdata_airspeed_node->getFloatValue() * 100);
    *(int16_t *)buf = airspeed; buf += 2;

    float alt = airdata_altitude_node->getFloatValue();
    // *(float *)buf = alt; buf += 4;
    memcpy( buf, &alt, 4 ); buf+= 4;

    float alt_true = airdata_altitude_true_node->getFloatValue();
    // *(float *)buf = alt_true; buf += 4;
    memcpy( buf, &alt_true, 4 ); buf+= 4;

    int16_t climb = (int16_t)((airdata_climb_fps_node->getFloatValue() * 60) * 10);
    *(int16_t *)buf = climb; buf += 2;

    int16_t empty = (int16_t)(0);
    *(int16_t *)buf = empty; buf += 2;

    uint16_t wind_deg = (uint16_t)(airdata_wind_dir_node->getFloatValue()*100);
    *(uint16_t *)buf = wind_deg; buf += 2;

    uint8_t wind_kts = (uint8_t)(airdata_wind_speed_node->getFloatValue() * 4);
    *buf = wind_kts; buf += 1;

    uint8_t pitot_scale = (uint8_t)(airdata_pitot_scale_node->getFloatValue()*100);
    *(uint8_t *)buf = pitot_scale; buf += 1;

    uint8_t status = airdata_status_node->getIntValue();
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

    double time = filter_timestamp_node->getDoubleValue();
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

    double lat = filter_lat_node->getDoubleValue();
    // *(double *)buf = lat; buf += 8;
    memcpy( buf, &lat, 8 ); buf += 8;

    double lon = filter_lon_node->getDoubleValue();
    // *(double *)buf = lon; buf += 8;
    memcpy( buf, &lon, 8 ); buf += 8;

    float alt = filter_alt_node->getFloatValue();
    // *(float *)buf = alt; buf += 4;
    memcpy( buf, &alt, 4 ); buf += 4;

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

    int8_t seq = (int8_t)link_seq_num->getIntValue();
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
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

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
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

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


int UGPacketizer::packetize_ap( uint8_t *buf, uint8_t route_size,
				SGWayPoint *wp, int index )
{
    uint8_t *startbuf = buf;

    double time = get_Time();
    // *(double *)buf = time; buf += 8;
    memcpy( buf, &time, 8 ); buf += 8;

    int16_t hdg = (int16_t)(ap_hdg->getFloatValue() * 10.0);
    *(int16_t *)buf = hdg; buf += 2;

    int16_t roll = (int16_t)(ap_roll->getFloatValue() * 10.0);
    *(int16_t *)buf = roll; buf += 2;

    // compute target AP msl as:
    //   ground_alt(pressure) + altitude_agl(press) - error(press)
    float target_agl_ft = ap_altitude_agl->getFloatValue();
    float ground_m = pressure_ground_alt_node->getFloatValue();
    float error_m = pressure_error_node->getFloatValue();
    float alt_msl_ft = (ground_m + error_m) * SG_METER_TO_FEET + target_agl_ft;
    *(uint16_t *)buf = (uint16_t)alt_msl_ft; buf += 2;

    int16_t climb = (int16_t)(ap_climb->getFloatValue() * 10.0);
    *(int16_t *)buf = climb; buf += 2;

    int16_t pitch = (int16_t)(ap_pitch->getFloatValue() * 10.0);
    *(int16_t *)buf = pitch; buf += 2;

    int16_t theta_dot = (int16_t)(ap_theta_dot->getFloatValue() * 1000.0);
    *(int16_t *)buf = theta_dot; buf += 2;

    int16_t speed = (int16_t)(ap_speed->getFloatValue() * 10.0);
    *(int16_t *)buf = speed; buf += 2;

    int target_wpt_index = ap_waypoint->getIntValue();
    if ( target_wpt_index >= route_size ) {
	target_wpt_index = 0;
    }
    uint16_t waypoint = (uint16_t)target_wpt_index;
    *(uint16_t *)buf = waypoint; buf += 2;

    // *(double *)buf = wp->get_target_lon(); buf += 8;
    double tlon = wp->get_target_lon();
    memcpy( buf, &tlon, 8 ); buf += 8;
    // *(double *)buf = wp->get_target_lat(); buf += 8;
    double tlat = wp->get_target_lat();
    memcpy( buf, &tlat, 8 ); buf += 8;
    *(uint16_t *)buf = index; buf += 2;
    *(uint16_t *)buf = route_size; buf += 2;

    int8_t seq = (int8_t)link_seq_num->getIntValue();
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

    uint16_t loadavg = (uint16_t)(system_loadavg_node->getFloatValue() * 100.0);
    *(uint16_t *)buf = loadavg; buf += 2;

    uint16_t vcc = (uint16_t)(avionics_vcc_node->getFloatValue() * 1000.0);
    *(uint16_t *)buf = vcc; buf += 2;

    uint16_t volt = (uint16_t)(extern_volt_node->getFloatValue() * 1000.0);
    *(uint16_t *)buf = volt; buf += 2;

    uint16_t cell_volt = (uint16_t)(extern_cell_volt_node->getFloatValue() * 1000.0);
    *(uint16_t *)buf = cell_volt; buf += 2;

    uint16_t amp = (uint16_t)(extern_amp_node->getFloatValue() * 1000.0);
    *(uint16_t *)buf = amp; buf += 2;

    uint16_t mah = (uint16_t)(extern_mah_node->getFloatValue());
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

    uint16_t trigger_num = (uint16_t)(payload_trigger_num_node->getIntValue());
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

    double filter_hdg = (SGD_PI * 0.5 - atan2(filter_vn_node->getDoubleValue(), filter_ve_node->getDoubleValue())) * SG_RADIANS_TO_DEGREES;
    snprintf(buf, max_buf, "%.2f,%.1f,%.1f,%.1f,%.1f,%.2f",
	     imu_timestamp_node->getDoubleValue(),
	     ap_hdg->getDoubleValue(),
	     ap_roll->getDoubleValue(),
	     filter_hdg,
	     filter_phi_node->getDoubleValue(),
	     act_aileron_node->getDoubleValue());

    return buf;
}

string UGPacketizer::get_fcs_speed_string() {
    static int max_buf = 256;
    char buf[max_buf];

    snprintf(buf, max_buf, "%.2f,%.1f,%.1f,%.1f,%.1f,%.2f",
	     imu_timestamp_node->getDoubleValue(),
	     ap_speed->getDoubleValue(),
	     ap_pitch->getDoubleValue(),
	     airdata_airspeed_node->getDoubleValue(),
	     filter_theta_node->getDoubleValue(),
	     act_elevator_node->getDoubleValue());

    return buf;
}

string UGPacketizer::get_fcs_altitude_string() {
    static int max_buf = 256;
    char buf[max_buf];

    snprintf(buf, max_buf, "%.2f,%.1f,%.1f,%.2f",
	     imu_timestamp_node->getDoubleValue(),
	     ap_altitude_agl->getDoubleValue(),
	     official_altitude_agl_node->getDoubleValue(),
	     act_throttle_node->getDoubleValue() );

    return buf;
}

bool UGPacketizer::decode_fcs_update(vector <string> tokens) {
    if ( tokens.size() > 0 && tokens[0] == "fcs-update" ) {
	// remove initial keyword if needed
	tokens.erase(tokens.begin());
    }

    if ( tokens.size() == 9 ) {
	int i = atoi(tokens[0].c_str());
	SGPropertyNode *pid
	    = fgGetNode("/config/fcs/autopilot/pid-controller",i);
	if ( pid == NULL ) {
	    return false;
	}

	SGPropertyNode *config = pid->getNode("config");
	if ( config == NULL ) {
	    return false;
	}

	SGPropertyNode *Kp = config->getNode("Kp", true);
	Kp->setDoubleValue( atof(tokens[1].c_str()) );
	SGPropertyNode *beta = config->getNode("beta", true);
	beta->setDoubleValue( atof(tokens[2].c_str()) );
	SGPropertyNode *alpha = config->getNode("alpha", true);
	alpha->setDoubleValue( atof(tokens[3].c_str()) );
	SGPropertyNode *gamma = config->getNode("gamma", true);
	gamma->setDoubleValue( atof(tokens[4].c_str()) );
	SGPropertyNode *Ti = config->getNode("Ti", true);
	Ti->setDoubleValue( atof(tokens[5].c_str()) );
	SGPropertyNode *Td = config->getNode("Td", true);
	Td->setDoubleValue( atof(tokens[6].c_str()) );
	SGPropertyNode *min = config->getNode("u_min", true);
	min->setDoubleValue( atof(tokens[7].c_str()) );
	SGPropertyNode *max = config->getNode("u_max", true);
	max->setDoubleValue( atof(tokens[8].c_str()) );

	return true;
    } else if ( tokens.size() == 5 ) {
	int i = atoi(tokens[0].c_str());
	SGPropertyNode *pid
	    = fgGetNode("/config/fcs/autopilot/pi-simple-controller",i);
	if ( pid == NULL ) {
	    return false;
	}

	SGPropertyNode *config = pid->getNode("config");
	if ( config == NULL ) {
	    return false;
	}

	SGPropertyNode *Kp = config->getNode("Kp", true);
	Kp->setDoubleValue( atof(tokens[1].c_str()) );
	SGPropertyNode *Ki = config->getNode("Ki", true);
	Ki->setDoubleValue( atof(tokens[2].c_str()) );
	SGPropertyNode *min = config->getNode("u_min", true);
	min->setDoubleValue( atof(tokens[3].c_str()) );
	SGPropertyNode *max = config->getNode("u_max", true);
	max->setDoubleValue( atof(tokens[4].c_str()) );

	return true;
     } else {
	return false;
    }
}

#include <stdint.h>

#include <stdio.h>

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
    gps_status_node = NULL;
}

// initialize imu property nodes 
void UGPacketizer::bind_imu_nodes() {
    imu_timestamp_node = fgGetNode("/sensors/imu/timestamp", true);
    imu_p_node = fgGetNode("/sensors/imu/p-rad_sec", true);
    imu_q_node = fgGetNode("/sensors/imu/q-rad_sec", true);
    imu_r_node = fgGetNode("/sensors/imu/r-rad_sec", true);
    imu_ax_node = fgGetNode("/sensors/imu/ax-mps_sec", true);
    imu_ay_node = fgGetNode("/sensors/imu/ay-mps_sec", true);
    imu_az_node = fgGetNode("/sensors/imu/az-mps_sec", true);
    imu_hx_node = fgGetNode("/sensors/imu/hx", true);
    imu_hy_node = fgGetNode("/sensors/imu/hy", true);
    imu_hz_node = fgGetNode("/sensors/imu/hz", true);
    imu_status_node = NULL;
}

// initialize filter property nodes
void UGPacketizer::bind_filter_nodes() {
    filter_timestamp_node = fgGetNode("/filters/filter/timestamp", true);
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
    act_timestamp_node = fgGetNode("/actuators/actuator/timestamp", true);
    act_aileron_node = fgGetNode("/actuators/actuator/channel", 0, true);
    act_elevator_node = fgGetNode("/actuators/actuator/channel", 1, true);
    act_throttle_node = fgGetNode("/actuators/actuator/channel", 2, true);
    act_rudder_node = fgGetNode("/actuators/actuator/channel", 3, true);
    act_channel5_node = fgGetNode("/actuators/actuator/channel", 4, true);
    act_channel6_node = fgGetNode("/actuators/actuator/channel", 5, true);
    act_channel7_node = fgGetNode("/actuators/actuator/channel", 6, true);
    act_channel8_node = fgGetNode("/actuators/actuator/channel", 7, true);
}


UGPacketizer::UGPacketizer() {
    bind_gps_nodes();
    bind_imu_nodes();
    bind_filter_nodes();
    bind_actuator_nodes();
}


int UGPacketizer::packetize_gps( uint8_t *buf ) {
    uint8_t *startbuf = buf;

    double time = gps_timestamp_node->getDoubleValue();
    *(double *)buf = time; buf += 8;

    double lat = gps_lat_node->getDoubleValue();
    *(double *)buf = lat; buf += 8;

    double lon = gps_lon_node->getDoubleValue();
    *(double *)buf = lon; buf += 8;

    /* resolution of 0.001 meters */
    int32_t alt = (int)(gps_alt_node->getDoubleValue() * 1000);
    *(int32_t *)buf = alt; buf += 4;

    /* +/- 327.67 mps (732.9 mph), resolution of 0.01 mps */
    int16_t vn = (int16_t)(gps_vn_node->getDoubleValue() * 100);
    *(int16_t *)buf = vn; buf += 2;

    int16_t ve = (int16_t)(gps_ve_node->getDoubleValue() * 100);
    *(int16_t *)buf = ve; buf += 2;

    int16_t vd = (int16_t)(gps_vd_node->getDoubleValue() * 100);
    *(int16_t *)buf = vd; buf += 2;
    
    double date = gps_unix_sec_node->getDoubleValue();
    *(double *)buf = date; buf += 8;

    uint8_t status = 0;
    *buf = status; buf++;

    return buf - startbuf;
}


void UGPacketizer::decode_gps( uint8_t *buf ) {
    double time = *(double *)buf; buf += 8;
    double lat = *(double *)buf; buf += 8;
    double lon = *(double *)buf; buf += 8;
    int32_t alt = *(int32_t *)buf; buf += 4;
    int16_t vn = *(int16_t *)buf; buf += 2;
    int16_t ve = *(int16_t *)buf; buf += 2;
    int16_t vd = *(int16_t *)buf; buf += 2;
    double date = *(double *)buf; buf += 8;
    uint8_t status = *(uint8_t *)buf; buf += 1;

    printf("t = %.2f (%.8f %.8f) a=%.2f  (%.2f %.2f %.2f) %.2f %d\n",
	   time, lat, lon, alt/1000.0, vn/100.0, ve/100.0, vd/100.0, date,
	   status);
}


int UGPacketizer::packetize_imu( uint8_t *buf ) {
    uint8_t *startbuf = buf;

    double time = imu_timestamp_node->getDoubleValue();
    *(double *)buf = time; buf += 8;

    int32_t p = (int)(imu_p_node->getDoubleValue() * 1000);
    *(int32_t *)buf = p; buf += 4;

    int32_t q = (int)(imu_q_node->getDoubleValue() * 1000);
    *(int32_t *)buf = q; buf += 4;

    int32_t r = (int)(imu_r_node->getDoubleValue() * 1000);
    *(int32_t *)buf = r; buf += 4;

    int32_t ax = (int)(imu_ax_node->getDoubleValue() * 1000);
    *(int32_t *)buf = ax; buf += 4;

    int32_t ay = (int)(imu_ay_node->getDoubleValue() * 1000);
    *(int32_t *)buf = ay; buf += 4;

    int32_t az = (int)(imu_az_node->getDoubleValue() * 1000);
    *(int32_t *)buf = az; buf += 4;

    int32_t hx = (int)(imu_hx_node->getDoubleValue() * 1000);
    *(int32_t *)buf = hx; buf += 4;

    int32_t hy = (int)(imu_hy_node->getDoubleValue() * 1000);
    *(int32_t *)buf = hy; buf += 4;

    int32_t hz = (int)(imu_hz_node->getDoubleValue() * 1000);
    *(int32_t *)buf = hz; buf += 4;

    uint8_t status = 0;
    *buf = status; buf++;

    return buf - startbuf;
}


void UGPacketizer::decode_imu( uint8_t *buf ) {
    double time = *(double *)buf; buf += 8;
    double p = *(int32_t *)buf; buf += 4;
    double q = *(int32_t *)buf; buf += 4;
    double r = *(int32_t *)buf; buf += 4;
    double ax = *(int32_t *)buf; buf += 4;
    double ay = *(int32_t *)buf; buf += 4;
    double az = *(int32_t *)buf; buf += 4;
    double hx = *(int32_t *)buf; buf += 4;
    double hy = *(int32_t *)buf; buf += 4;
    double hz = *(int32_t *)buf; buf += 4;
    uint8_t status = *(uint8_t *)buf; buf += 1;

    printf("t = %.2f (%.3f %.3f %.3f) (%.3f %.3f %.f) (%.3f %.3f %.3f) %d\n",
	   time,
	   p/1000.0, q/1000.0, r/1000.0,
	   ax/1000.0, ay/1000.0, az/1000.0,
	   hx/1000.0, hy/1000.0, hz/1000.0,
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

    /* resolution of 0.001 meters */
    int32_t alt = (int)(filter_alt_node->getDoubleValue() * 1000);
    *(int32_t *)buf = alt; buf += 4;

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

    uint8_t status = 0;
    *buf = status; buf++;

    return buf - startbuf;
}


void UGPacketizer::decode_filter( uint8_t *buf ) {
    double time = *(double *)buf; buf += 8;
    double lat = *(double *)buf; buf += 8;
    double lon = *(double *)buf; buf += 8;
    int32_t alt = *(int32_t *)buf; buf += 4;
    int16_t vn = *(int16_t *)buf; buf += 2;
    int16_t ve = *(int16_t *)buf; buf += 2;
    int16_t vd = *(int16_t *)buf; buf += 2;
    int16_t phi = *(int16_t *)buf; buf += 2;
    int16_t the = *(int16_t *)buf; buf += 2;
    int16_t psi = *(int16_t *)buf; buf += 2;
    uint8_t status = *(uint8_t *)buf; buf += 1;

    printf("t = %.2f (%.8f %.8f a=%.2f) (%.2f %.2f %.2f) (%.1f %.1f %.1f) %d\n",
	   time,
	   lat, lon, alt/1000.0,
	   vn/100.0, ve/100.0, vd/100.0,
	   phi/10.0, the/10.0, psi/10.0,
	   status);
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



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
}


UGPacketizer::UGPacketizer() {
    bind_gps_nodes();
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

    printf("t = %.2f (%.8f %.8f) a=%.2f  (%.2f %.2f %.2f) %.2f\n",
	   time, lat, lon, alt/1000.0, vn/100.0, ve/100.0, vd/100.0, date );
}

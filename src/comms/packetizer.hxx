#ifndef _AURA_PACKETIZER_HXX
#define _AURA_PACKETIZER_HXX

#include "python/pyprops.hxx"

#include <stdint.h>

#include "control/route_mgr.hxx"


#define START_OF_MSG0 147	// 0x93
#define START_OF_MSG1 224	// 0xE0

enum ugPacketType {
    GPS_PACKET_V1 = 0,
    IMU_PACKET_V1 = 1,
    FILTER_PACKET_V1 = 2,
    ACTUATOR_PACKET_V1 = 3,
    PILOT_INPUT_PACKET_V1 = 4,
    AP_STATUS_PACKET_V1 = 5,
    AIRDATA_PACKET_V3 = 9,
    AP_STATUS_PACKET_V2 = 10,
    SYSTEM_HEALTH_PACKET_V2 = 11,
    PAYLOAD_PACKET_V1 = 12,
    AIRDATA_PACKET_V4 = 13,
    SYSTEM_HEALTH_PACKET_V3 = 14,
    IMU_PACKET_V2 = 15,
    GPS_PACKET_V2 = 16,
    IMU_PACKET_V3 = 17,
    AIRDATA_PACKET_V5 = 18,
    SYSTEM_HEALTH_PACKET_V4 = 19,
    PILOT_INPUT_PACKET_V2 = 20,
    ACTUATOR_PACKET_V2 = 21,
};


class UGPacketizer {
    // property nodes
    pyPropertyNode gps_node;
    pyPropertyNode imu_node;
    pyPropertyNode airdata_node;
    pyPropertyNode pos_node;
    pyPropertyNode pos_pressure_node;
    pyPropertyNode pos_combined_node;
    pyPropertyNode vel_node;
    pyPropertyNode filter_node;
    pyPropertyNode wind_node;
    pyPropertyNode act_node;
    pyPropertyNode pilot_node;
    pyPropertyNode targets_node;
    pyPropertyNode route_node;
    pyPropertyNode status_node;
    pyPropertyNode apm2_node;
    pyPropertyNode payload_node;
    pyPropertyNode remote_link_node;

public:

    UGPacketizer();
    inline ~UGPacketizer() {}

    int packetize_gps( uint8_t *buf );
    void decode_gps( uint8_t *buf );

    int packetize_imu( uint8_t *buf );
    void decode_imu( uint8_t *buf );

    int packetize_airdata( uint8_t *buf );
    void decode_airdata( uint8_t *buf );

    int packetize_filter( uint8_t *buf );
    void decode_filter( uint8_t *buf );

    int packetize_actuator( uint8_t *buf );
    void decode_actuator( uint8_t *buf );

    int packetize_pilot( uint8_t *buf );
    void decode_pilot( uint8_t *buf );

    int packetize_ap( uint8_t *buf, uint8_t route_size, SGWayPoint *wp,
		      int index );
    void decode_ap( uint8_t *buf );

    int packetize_health( uint8_t *buf );
    void decode_health( uint8_t *buf );

    int packetize_payload( uint8_t *buf );
    void decode_payload( uint8_t *buf );

    // packetizer has node references to all the good data setup
    // already, so let's use it to generate other data strings as
    // well.
    string get_fcs_nav_string();
    string get_fcs_speed_string();
    string get_fcs_altitude_string();
    bool decode_fcs_update(vector <string> tokens);
};


#endif // _AURA_PACKETIZER_HXX

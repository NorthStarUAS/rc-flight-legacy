#ifndef UG_PACKETIZER_HXX
#define UG_PACKETIZER_HXX


#include <stdint.h>

#include "control/route_mgr.hxx"
#include "props/props.hxx"


#define START_OF_MSG0 147	// 0x93
#define START_OF_MSG1 224	// 0xE0

enum ugPacketType {
    GPS_PACKET_V1 = 0,
    IMU_PACKET_V1 = 1,
    FILTER_PACKET_V1 = 2,
    ACTUATOR_PACKET_V1 = 3,
    PILOT_INPUT_PACKET_V1 = 4,
    AP_STATUS_PACKET_V1 = 5,
    AIR_DATA_PACKET_V1 = 6,
    SYSTEM_HEALTH_PACKET_V1 = 7,
    AIR_DATA_PACKET_V2 = 8,
    AIR_DATA_PACKET_V3 = 9,
    AP_STATUS_PACKET_V2 = 10,
    SYSTEM_HEALTH_PACKET_V2 = 11,
    PAYLOAD_PACKET_V1 = 12,
    AIR_DATA_PACKET_V4 = 13
};


class UGPacketizer {

    // gps property nodes
    SGPropertyNode *gps_timestamp_node;
    SGPropertyNode *gps_lat_node;
    SGPropertyNode *gps_lon_node;
    SGPropertyNode *gps_alt_node;
    SGPropertyNode *gps_vn_node;
    SGPropertyNode *gps_ve_node;
    SGPropertyNode *gps_vd_node;
    SGPropertyNode *gps_unix_sec_node;
    SGPropertyNode *gps_satellites_node;
    SGPropertyNode *gps_status_node;

    // imu property nodes
    SGPropertyNode *imu_timestamp_node;
    SGPropertyNode *imu_p_node;
    SGPropertyNode *imu_q_node;
    SGPropertyNode *imu_r_node;
    SGPropertyNode *imu_ax_node;
    SGPropertyNode *imu_ay_node;
    SGPropertyNode *imu_az_node;
    SGPropertyNode *imu_hx_node;
    SGPropertyNode *imu_hy_node;
    SGPropertyNode *imu_hz_node;
    SGPropertyNode *imu_status_node;

    // air data property nodes
    SGPropertyNode *airdata_timestamp_node;
    SGPropertyNode *airdata_pressure_node;
    SGPropertyNode *airdata_temperature_node;
    SGPropertyNode *airdata_altitude_node;
    SGPropertyNode *airdata_altitude_true_node;
    SGPropertyNode *official_altitude_agl_node;
    SGPropertyNode *airdata_airspeed_node;
    SGPropertyNode *airdata_climb_fps_node;
    SGPropertyNode *airdata_wind_dir_node;
    SGPropertyNode *airdata_wind_speed_node;
    SGPropertyNode *airdata_pitot_scale_node;
    SGPropertyNode *airdata_status_node;

    // filter property nodes
    SGPropertyNode *filter_timestamp_node;
    SGPropertyNode *filter_theta_node;
    SGPropertyNode *filter_phi_node;
    SGPropertyNode *filter_psi_node;
    SGPropertyNode *filter_lat_node;
    SGPropertyNode *filter_lon_node;
    SGPropertyNode *filter_alt_node;
    SGPropertyNode *filter_vn_node;
    SGPropertyNode *filter_ve_node;
    SGPropertyNode *filter_vd_node;
    SGPropertyNode *filter_status_node;

    // actuator property nodes
    SGPropertyNode *act_timestamp_node;
    SGPropertyNode *act_aileron_node;
    SGPropertyNode *act_elevator_node;
    SGPropertyNode *act_throttle_node;
    SGPropertyNode *act_rudder_node;
    SGPropertyNode *act_channel5_node;
    SGPropertyNode *act_channel6_node;
    SGPropertyNode *act_channel7_node;
    SGPropertyNode *act_channel8_node;
    SGPropertyNode *act_status_node;

    // pilot input property nodes
    SGPropertyNode *pilot_timestamp_node;
    SGPropertyNode *pilot_aileron_node;
    SGPropertyNode *pilot_elevator_node;
    SGPropertyNode *pilot_throttle_node;
    SGPropertyNode *pilot_rudder_node;
    SGPropertyNode *pilot_channel5_node;
    SGPropertyNode *pilot_channel6_node;
    SGPropertyNode *pilot_channel7_node;
    SGPropertyNode *pilot_channel8_node;
    SGPropertyNode *pilot_status_node;

    // autopilot status nodes
    SGPropertyNode *pressure_ground_alt_node;
    SGPropertyNode *pressure_error_node;
    SGPropertyNode *ap_hdg;
    SGPropertyNode *ap_roll;
    SGPropertyNode *ap_altitude_agl;
    SGPropertyNode *ap_altitude_msl;
    SGPropertyNode *ap_climb;
    SGPropertyNode *ap_pitch;
    SGPropertyNode *ap_theta_dot;
    SGPropertyNode *ap_speed;
    SGPropertyNode *ap_waypoint;

    // system health nodes
    SGPropertyNode *system_loadavg_node;
    SGPropertyNode *avionics_vcc_node;
    SGPropertyNode *extern_volt_node;
    SGPropertyNode *extern_amp_node;
    SGPropertyNode *extern_mah_node;

    // payload status nodes
    SGPropertyNode *payload_trigger_num_node;

    // console link nodes
    SGPropertyNode *link_seq_num;

    void bind_gps_nodes();
    void bind_imu_nodes();
    void bind_airdata_nodes();
    void bind_filter_nodes();
    void bind_actuator_nodes();
    void bind_pilot_nodes();
    void bind_ap_nodes();
    void bind_health_nodes();
    void bind_payload_nodes();

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


#endif // UG_PACKETIZER_HXX

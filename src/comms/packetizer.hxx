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
    AIR_DATA_PACKET_V1 = 6,
    SYSTEM_HEALTH_PACKET_V1 = 7,
    AIR_DATA_PACKET_V2 = 8,
    AIR_DATA_PACKET_V3 = 9,
    AP_STATUS_PACKET_V2 = 10,
    SYSTEM_HEALTH_PACKET_V2 = 11,
    PAYLOAD_PACKET_V1 = 12,
    AIR_DATA_PACKET_V4 = 13,
    SYSTEM_HEALTH_PACKET_V3 = 14,
    IMU_PACKET_V2 = 15,
};


class UGPacketizer {

    // gps property nodes
    pyPropertyNode gps_timestamp_node;
    pyPropertyNode gps_lat_node;
    pyPropertyNode gps_lon_node;
    pyPropertyNode gps_alt_node;
    pyPropertyNode gps_vn_node;
    pyPropertyNode gps_ve_node;
    pyPropertyNode gps_vd_node;
    pyPropertyNode gps_unix_sec_node;
    pyPropertyNode gps_satellites_node;
    pyPropertyNode gps_status_node;

    // imu property nodes
    pyPropertyNode imu_timestamp_node;
    pyPropertyNode imu_p_node;
    pyPropertyNode imu_q_node;
    pyPropertyNode imu_r_node;
    pyPropertyNode imu_ax_node;
    pyPropertyNode imu_ay_node;
    pyPropertyNode imu_az_node;
    pyPropertyNode imu_hx_node;
    pyPropertyNode imu_hy_node;
    pyPropertyNode imu_hz_node;
    pyPropertyNode imu_temp_node;
    pyPropertyNode imu_status_node;

    // air data property nodes
    pyPropertyNode airdata_timestamp_node;
    pyPropertyNode airdata_pressure_node;
    pyPropertyNode airdata_temperature_node;
    pyPropertyNode airdata_altitude_node;
    pyPropertyNode airdata_altitude_true_node;
    pyPropertyNode official_altitude_agl_node;
    pyPropertyNode airdata_airspeed_node;
    pyPropertyNode airdata_climb_fps_node;
    pyPropertyNode airdata_wind_dir_node;
    pyPropertyNode airdata_wind_speed_node;
    pyPropertyNode airdata_pitot_scale_node;
    pyPropertyNode airdata_status_node;

    // filter property nodes
    pyPropertyNode filter_timestamp_node;
    pyPropertyNode filter_theta_node;
    pyPropertyNode filter_phi_node;
    pyPropertyNode filter_psi_node;
    pyPropertyNode filter_lat_node;
    pyPropertyNode filter_lon_node;
    pyPropertyNode filter_alt_node;
    pyPropertyNode filter_vn_node;
    pyPropertyNode filter_ve_node;
    pyPropertyNode filter_vd_node;
    pyPropertyNode filter_status_node;

    // actuator property nodes
    pyPropertyNode act_timestamp_node;
    pyPropertyNode act_aileron_node;
    pyPropertyNode act_elevator_node;
    pyPropertyNode act_throttle_node;
    pyPropertyNode act_rudder_node;
    pyPropertyNode act_channel5_node;
    pyPropertyNode act_channel6_node;
    pyPropertyNode act_channel7_node;
    pyPropertyNode act_channel8_node;
    pyPropertyNode act_status_node;

    // pilot input property nodes
    pyPropertyNode pilot_timestamp_node;
    pyPropertyNode pilot_aileron_node;
    pyPropertyNode pilot_elevator_node;
    pyPropertyNode pilot_throttle_node;
    pyPropertyNode pilot_rudder_node;
    pyPropertyNode pilot_channel5_node;
    pyPropertyNode pilot_channel6_node;
    pyPropertyNode pilot_channel7_node;
    pyPropertyNode pilot_channel8_node;
    pyPropertyNode pilot_status_node;

    // autopilot status nodes
    pyPropertyNode pressure_ground_alt_node;
    pyPropertyNode pressure_error_node;
    pyPropertyNode ap_hdg;
    pyPropertyNode ap_roll;
    pyPropertyNode ap_altitude_agl;
    pyPropertyNode ap_altitude_msl;
    pyPropertyNode ap_climb;
    pyPropertyNode ap_pitch;
    pyPropertyNode ap_theta_dot;
    pyPropertyNode ap_speed;
    pyPropertyNode ap_waypoint;

    // system health nodes
    pyPropertyNode system_loadavg_node;
    pyPropertyNode avionics_vcc_node;
    pyPropertyNode extern_volt_node;
    pyPropertyNode extern_cell_volt_node;
    pyPropertyNode extern_amp_node;
    pyPropertyNode extern_mah_node;

    // payload status nodes
    pyPropertyNode payload_trigger_num_node;

    // console link nodes
    pyPropertyNode link_seq_num;

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


#endif // _AURA_PACKETIZER_HXX

#ifndef UG_PACKETIZER_HXX
#define UG_PACKETIZER_HXX


#include <stdint.h>

#include "control/route_mgr.hxx"
#include "props/props.hxx"


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
    SGPropertyNode *ap_hdg;
    SGPropertyNode *ap_roll;
    SGPropertyNode *ap_altitude;
    SGPropertyNode *ap_climb;
    SGPropertyNode *ap_pitch;
    SGPropertyNode *ap_speed;
    SGPropertyNode *ap_waypoint;

    // system health nodes
    SGPropertyNode *system_load_avg;

    // console link nodes
    SGPropertyNode *console_seq_num;

    void bind_gps_nodes();
    void bind_imu_nodes();
    void bind_filter_nodes();
    void bind_actuator_nodes();
    void bind_pilot_nodes();
    void bind_ap_nodes();
    void bind_health_nodes();

public:

    UGPacketizer();
    inline ~UGPacketizer() {}

    int packetize_gps( uint8_t *buf );
    void decode_gps( uint8_t *buf );

    int packetize_imu( uint8_t *buf );
    void decode_imu( uint8_t *buf );

    int packetize_filter( uint8_t *buf );
    void decode_filter( uint8_t *buf );

    int packetize_actuator( uint8_t *buf );
    void decode_actuator( uint8_t *buf );

    int packetize_pilot( uint8_t *buf );
    void decode_pilot( uint8_t *buf );

    int packetize_ap( uint8_t *buf, SGWayPoint *wp, int index );
    void decode_ap( uint8_t *buf );

    int packetize_health( uint8_t *buf );
    void decode_health( uint8_t *buf );

};


#endif // UG_PACKETIZER_HXX

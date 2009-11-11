#ifndef UG_PACKETIZER_HXX
#define UG_PACKETIZER_HXX


#include <stdint.h>

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

    void bind_gps_nodes();
    void bind_imu_nodes();
    void bind_filter_nodes();

public:

    UGPacketizer();
    inline ~UGPacketizer() {}

    int packetize_gps( uint8_t *buf );
    void decode_gps( uint8_t *buf );

    int packetize_imu( uint8_t *buf );
    void decode_imu( uint8_t *buf );

    int packetize_filter( uint8_t *buf );
    void decode_filter( uint8_t *buf );

};


#endif // UG_PACKETIZER_HXX

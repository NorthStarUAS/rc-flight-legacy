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

    void bind_gps_nodes();

public:

    UGPacketizer();
    inline ~UGPacketizer() {}

    int packetize_gps( uint8_t *buf );
    void decode_gps( uint8_t *buf );
};

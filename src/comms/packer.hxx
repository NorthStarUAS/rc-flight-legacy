#ifndef _AURA_PACKER_HXX
#define _AURA_PACKER_HXX

// import a python module and call it's init() and update() routines
// requires imported python modules to follow some basic rules to play
// nice.  (see examples in the code for now.)

#include "python/pymodule.hxx"
#include "python/pyprops.hxx"

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
    FILTER_PACKET_V2 = 22,
    PAYLOAD_PACKET_V2 = 23,
    AP_STATUS_PACKET_V3 = 24,
};

class pyModulePacker: public pyModuleBase {

public:

    // constructor / destructor
    pyModulePacker();
    ~pyModulePacker() {}

    int pack(int index, const char *pack_function, uint8_t *buf);
    int pack_gps(int index, uint8_t *buf);
    int pack_imu(int index, uint8_t *buf);
    int pack_airdata(int index, uint8_t *buf);
    int pack_health(int index, uint8_t *buf);
    int pack_pilot(int index, uint8_t *buf);
    int pack_actuator(int index, uint8_t *buf);
    int pack_filter(int index, uint8_t *buf);
    int pack_payload(int index, uint8_t *buf);
    int pack_ap(int index, uint8_t *buf);
};

#endif // _AURA_PACKER_HXX

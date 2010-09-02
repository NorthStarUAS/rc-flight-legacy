#ifndef _UGEAR_CONSOLE_LINK_H
#define _UGEAR_CONSOLE_LINK_H


#include <stdint.h>


#define START_OF_MSG0 147	// 0x93
#define START_OF_MSG1 224	// 0xE0

enum ugPacketType {
    GPS_PACKET_V1 = 0x00,
    IMU_PACKET_V1 = 0x01,
    FILTER_PACKET_V1 = 0x02,
    ACTUATOR_PACKET_V1 = 0x03,
    PILOT_INPUT_PACKET_V1 = 0x04,
    AP_STATUS_PACKET_V1 = 0x05,
    AIR_DATA_PACKET_V1 = 0x06,
    SYSTEM_HEALTH_PACKET_V1 = 0x07,
};

extern bool console_link_on;

void console_link_init();
bool console_link_gps( uint8_t *buf, int size, int skip_count );
bool console_link_imu( uint8_t *buf, int size, int skip_count );
bool console_link_airdata( uint8_t *buf, int size, int skip_count );
bool console_link_filter( uint8_t *buf, int size, int skip_count );
bool console_link_actuator( uint8_t *buf, int size, int skip_count );
bool console_link_pilot( uint8_t *buf, int size, int skip_count );
bool console_link_ap( uint8_t *buf, int size, int skip_count );
bool console_link_command();


#endif // _UGEAR_CONSOLE_LINK_H

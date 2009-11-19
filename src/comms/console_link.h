#ifndef _UGEAR_CONSOLE_LINK_H
#define _UGEAR_CONSOLE_LINK_H


#include <stdint.h>


#define START_OF_MSG0 147	// 0x93
#define START_OF_MSG1 224	// 0xE0

enum ugPacketType {
    GPS_PACKET_V1 = 0,
    IMU_PACKET_V1 = 1,
    FILTER_PACKET_V1 = 2,
    SERVO_PACKET_V1 = 3,
    HEALTH_PACKET_V1 = 4
};

extern bool console_link_on;

void console_link_init();
void console_link_gps( uint8_t *gps_buf, int gps_size );
void console_link_imu( uint8_t *imu_buf, int imu_size );
void console_link_filter( uint8_t *filter_buf, int filter_size );
void console_link_servo( struct servo *servopacket );
void console_link_health( struct health *healthpacket );
bool console_link_command();


#endif // _UGEAR_CONSOLE_LINK_H

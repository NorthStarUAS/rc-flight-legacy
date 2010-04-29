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
    HEALTH_PACKET_V1 = 0x04,
    PILOT_INPUT_PACKET_V1 = 0x05
};

extern bool console_link_on;

void console_link_init();
void console_link_gps( uint8_t *gps_buf, int gps_size, int skip_count );
void console_link_imu( uint8_t *imu_buf, int imu_size, int skip_count );
void console_link_filter( uint8_t *filter_buf, int filter_size,
			  int skip_count );
void console_link_actuator( uint8_t *actuator_buf, int actuator_size,
			    int skip_count );
void console_link_pilot( uint8_t *pilot_buf, int pilot_size, int skip_count );
void console_link_health( struct health *healthpacket, int skip_count );
bool console_link_command();


#endif // _UGEAR_CONSOLE_LINK_H

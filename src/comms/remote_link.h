#ifndef _UGEAR_REMOTE_LINK_H
#define _UGEAR_REMOTE_LINK_H


#include <stdint.h>


extern bool remote_link_on;

void remote_link_init();
bool remote_link_gps( uint8_t *buf, int size, int skip_count );
bool remote_link_imu( uint8_t *buf, int size, int skip_count );
bool remote_link_airdata( uint8_t *buf, int size, int skip_count );
bool remote_link_filter( uint8_t *buf, int size, int skip_count );
bool remote_link_actuator( uint8_t *buf, int size, int skip_count );
bool remote_link_pilot( uint8_t *buf, int size, int skip_count );
bool remote_link_ap( uint8_t *buf, int size, int skip_count );
bool remote_link_health( uint8_t *buf, int size, int skip_count );
bool remote_link_payload( uint8_t *buf, int size, int skip_count );
bool remote_link_command();
void remote_link_flush_serial();


#endif // _UGEAR_REMOTE_LINK_H

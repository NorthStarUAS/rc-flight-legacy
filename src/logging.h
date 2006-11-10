#ifndef _UGEAR_LOGGING_H
#define _UGEAR_LOGGING_H


#include "globaldefs.h"


extern bool log_to_file;

bool logging_init();
bool logging_close();

void log_gps( struct gps *gpspacket );
void log_imu( struct imu *imupacket );
void log_nav( struct nav *navpacket );
void log_servo( struct servo *servopacket );

void ugear_cksum( uint8_t size, unsigned char *buf,
                  uint8_t *cksum0, uint8_t *cksum1 );


#endif // _UGEAR_LOGGING_H

#ifndef _UGEAR_LOGGING_H
#define _UGEAR_LOGGING_H


#include "globaldefs.h"


extern bool log_to_file;

bool logging_init();
bool logging_close();

void log_gps( struct gps *gpspacket );
void log_imu( struct imu *imupacket );
void log_nav( struct nav *navpacket );


#endif // _UGEAR_LOGGING_H

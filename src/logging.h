#ifndef UNAV_LOGGING_H
#define UNAV_LOGGING_H


#include "globaldefs.h"


extern short log_to_file;

bool logging_init();
bool logging_close();

void log_gps( struct gps *gpspacket );
void log_imu( struct imu *imupacket );
void log_nav( struct nav *navpacket );


#endif // UNAV_LOGGING_H

#ifndef _UGEAR_LOGGING_H
#define _UGEAR_LOGGING_H


#include "globaldefs.h"

#include "util/sg_path.hxx"

// global variables

extern bool log_to_file;
extern SGPath log_path;
extern bool display_on;

// global functions

bool logging_init();
bool logging_close();

void log_gps( struct gps *gpspacket );
void log_imu( struct imu *imupacket );
void log_nav( struct nav *navpacket );
void log_servo( struct servo *servopacket );
void log_health( struct health *healthpacket );

void flush_gps( );
void flush_imu( );
void flush_nav( );
void flush_servo( );
void flush_health( );

void display_message( struct imu *data, struct gps *gdata,
                      struct nav *ndata, struct servo *sdata,
                      struct health *hdata );


#endif // _UGEAR_LOGGING_H

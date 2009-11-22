#ifndef _UGEAR_LOGGING_H
#define _UGEAR_LOGGING_H

#include <stdint.h>

#include "include/globaldefs.h"

#include "util/sg_path.hxx"

// global variables

extern bool log_to_file;
extern SGPath log_path;
extern bool display_on;

// global functions

bool logging_init();
bool logging_close();

void log_gps( uint8_t *gps_buf, int gps_size, int skip_count );
void log_imu( uint8_t *imu_buf, int imu_size, int skip_count );
void log_filter( uint8_t *filter_buf, int filter_size, int skip_count );
void log_actuator( uint8_t *actuator_buf, int actuator_size, int skip_count );
void log_health( struct health *healthpacket, int skip_count );

void flush_gps();
void flush_imu();
void flush_filter();
void flush_actuator();
void flush_health();

void display_message( struct health *hdata );

bool logging_navstate_init();
void logging_navstate();
void logging_navstate_close();

#endif // _UGEAR_LOGGING_H

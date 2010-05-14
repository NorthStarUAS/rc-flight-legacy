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

void log_gps( uint8_t *buf, int size, int skip_count );
void log_imu( uint8_t *buf, int size, int skip_count );
void log_airdata( uint8_t *buf, int size, int skip_count );
void log_filter( uint8_t *buf, int size, int skip_count );
void log_actuator( uint8_t *buf, int size, int skip_count );
void log_pilot( uint8_t *buf, int size, int skip_count );
void log_ap( uint8_t *buf, int size, int skip_count );

void flush_gps();
void flush_imu();
void flush_airdata();
void flush_filter();
void flush_actuator();
void flush_pilot();
void flush_ap();

void display_message();

bool logging_navstate_init();
void logging_navstate();
void logging_navstate_close();

#endif // _UGEAR_LOGGING_H

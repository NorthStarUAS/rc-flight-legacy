#ifndef _AURA_LOGGING_H
#define _AURA_LOGGING_H

#include "python/pyprops.hxx"

#include <stdint.h>

#include "include/globaldefs.h"

#include "util/sg_path.hxx"

// global variables

extern bool log_to_file;
extern SGPath log_path;
extern bool event_log_on;

// global functions

bool logging_init();
bool logging_close();

void log_gps( uint8_t *buf, int size );
void log_imu( uint8_t *buf, int size, int skip_count );
void log_airdata( uint8_t *buf, int size, int skip_count );
void log_filter( uint8_t *buf, int size, int skip_count );
void log_actuator( uint8_t *buf, int size, int skip_count );
void log_pilot( uint8_t *buf, int size, int skip_count );
void log_ap( uint8_t *buf, int size, int skip_count );
void log_health( uint8_t *buf, int size, int skip_count );
void log_payload( uint8_t *buf, int size, int skip_count );

void flush_data();

bool event_log( const char *hdr, const char *msg );

bool log_imu_calibration( pyPropertyNode *config );


#endif // _AURA_LOGGING_H

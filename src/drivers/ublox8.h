/**
 * \file: gps_ublox8.h
 *
 * u-blox 7 protocol driver
 *
 * Copyright Curt Olson curtolson@flightgear.org
 *
 */

#pragma once

#include <pyprops.h>

#include "drivers/driver.h"

class ublox8_t: public driver_t {
    
public:
    ublox8_t() {}
    ~ublox8_t() {}
    void init( pyPropertyNode *config );
    float read();
    void process() {}
    void write() {};
    void close();
    void command( const char *cmd ) {}

private:
    pyPropertyNode gps_node;
    int fd = -1;
    bool open( const char *device_name, const int baud );
    bool read_ublox8();
    bool parse_msg( uint8_t msg_class, uint8_t msg_id,
                    uint16_t payload_length, uint8_t *payload );
};

/* #include "include/globaldefs.h" */

/* void gps_ublox8_init( string output_path, pyPropertyNode *config ); */
/* bool gps_ublox8_update(); */

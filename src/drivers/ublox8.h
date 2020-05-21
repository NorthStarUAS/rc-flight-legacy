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
    int state = 0;
    int msg_class = 0, msg_id = 0;
    int length_lo = 0, length_hi = 0, payload_length = 0;
    int counter = 0;
    uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    bool open( const char *device_name, const int baud );
    bool read_ublox8();
    bool parse_msg( uint8_t msg_class, uint8_t msg_id,
                    uint16_t payload_length, uint8_t *payload );
};

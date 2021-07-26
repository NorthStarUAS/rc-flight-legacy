//
// pololu maestro servo driver (usb/uart)
//

#pragma once

#include <props2.h>

#include "drivers/driver.h"

class maestro_t: public driver_t {
    
public:
    maestro_t() {}
    ~maestro_t() {}
    void init( PropertyNode *config );
    float read() { return 0.0; }
    void process() {}
    void write();
    void close();
    void command( const char *cmd ) {}

private:
    PropertyNode act_node;
    PropertyNode ap_node;
    PropertyNode pilot_node;
    static const int maestro_channels = 6;
    int fd = -1;
    float gains[maestro_channels] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    bool open( const char *device_name );
    void write_channel(int ch, float norm, bool symmetrical);
};

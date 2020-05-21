//
// pololu maestro servo driver (usb/uart)
//

#pragma once

#include <pyprops.h>

#include "drivers/driver.h"

class maestro_t: public driver_t {
    
public:
    maestro_t() {}
    ~maestro_t() {}
    void init( pyPropertyNode *config );
    float read() { return 0.0; }
    void process() {}
    void write();
    void close();
    void command( const char *cmd ) {}

private:
    pyPropertyNode act_node;
    pyPropertyNode ap_node;
    pyPropertyNode pilot_node;
    static const int maestro_channels = 6;
    int fd = -1;
    float gains[maestro_channels] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    bool open( const char *device_name );
    void write_channel(int ch, float norm, bool symmetrical);
};

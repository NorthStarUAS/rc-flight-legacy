//
// pololu maestro servo driver (usb/uart)
//

#pragma once

#include <string>
using std::string;

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
    const int maestro_channels = 6;
    pyPropertyNode act_node;
    int fd = -1;
    bool open( const char *device_name );
    void write_channel(int ch, float norm, bool symmetrical);
};
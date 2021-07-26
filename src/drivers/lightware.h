//
// lightware rangefinder driver (usb/uart)
//

#pragma once

#include <string>
using std::string;

#include <props2.h>

#include "drivers/driver.h"

class lightware_t: public driver_t {
    
public:
    lightware_t() {}
    ~lightware_t() {}
    void init( PropertyNode *config );
    float read();
    void process() {}
    void write() {}
    void close();
    void command( const char *cmd ) {}

private:
    PropertyNode pos_node;
    int fd = -1;
    bool open( const char *device_name );
};

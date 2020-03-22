// driver.h - base class for all drivers

#pragma once

#include <pyprops.h>

class driver_t {
public:
    driver_t() {}
    virtual ~driver_t() {}
    
    virtual void init( pyPropertyNode *config ) = 0;
    virtual float read() = 0;
    virtual void process() = 0;
    virtual void write() = 0;
    virtual void close() = 0;
    virtual void command(const char *cmd) = 0;
};

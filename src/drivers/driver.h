// driver.h - base class for all drivers

#pragma once

class driver_t {
public:
    driver_t();
    virtual ~driver_t() {}
    
    virtual void init() = 0;
    virtual void read() = 0;
    virtual void process() = 0;
    virtual void write() = 0;
    virtual void close() = 0;
};

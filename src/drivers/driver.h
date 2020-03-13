// driver.h - base class for all drivers

#pragma once

class Driver {
public:
    Driver();
    virtual ~Driver() {}
    
    virtual void init() = 0;
    virtual void read() = 0;
    virtual void process() = 0;
    virtual void write() = 0;
};

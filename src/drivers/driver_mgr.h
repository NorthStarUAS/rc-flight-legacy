// driver_mgr.h - class to load, manage, and execute drivers

#pragma once

#include <vector>
using std::vector;

#include <pyprops.h>

#include "driver.h"

class driver_mgr_t {
    
public:
    driver_mgr_t();
    ~driver_mgr_t() {}
    void init();
    float read();
    void process();
    void write();
    void close();
    void send_commands();

private:
    pyPropertyNode sensors_node;
    vector<driver_t *> drivers;
};

extern driver_mgr_t driver_mgr;

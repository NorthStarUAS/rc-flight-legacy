// driver_mgr.h - class to load, manage, and execute drivers

#pragma once

#include <vector>
using std::vector;

#include <props2.h>

#include "driver.h"

class driver_mgr_t {
    
public:
    driver_mgr_t();
    ~driver_mgr_t() {}
    void init(DocPointerWrapper d);
    float read();
    void process();
    void write();
    void close();
    void send_commands();

private:
    PropertyNode sensors_node;
    vector<driver_t *> drivers;
};

// driver_mgr.h - class to load, manage, and execute drivers

#pragma once

#include <vector>
using std::vector;

#include "driver.h"

class DriverMgr {
    
public:
    DriverMgr();
    void init();
    void update();
    
private:
    vector<Driver *> drivers;
};

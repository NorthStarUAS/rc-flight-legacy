#pragma once

#include <pyprops.h>
#include <pymodule.h>

#include "ap.h"

class control_t {
public:
    control_t() {};
    ~control_t() {};
    void init();
    void reset();
    void update( float dt );

private:
    pyModuleBase navigation;
    AuraAutopilot ap;
    
    pyPropertyNode status_node;
    pyPropertyNode ap_node;
    pyPropertyNode targets_node;
    pyPropertyNode tecs_node;
    pyPropertyNode task_node;
    pyPropertyNode pilot_node;
    pyPropertyNode flight_node;
    pyPropertyNode engine_node;
    pyPropertyNode route_node;
    pyPropertyNode active_node;
    pyPropertyNode home_node;
    pyPropertyNode circle_node;
    pyPropertyNode pos_node;

    void copy_pilot_inputs();
};

extern control_t control;

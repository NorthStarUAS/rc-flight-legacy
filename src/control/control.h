#pragma once

#include <props2.h>

#include "ap.h"

class control_t {
public:
    control_t() {};
    ~control_t() {};
    void init(DocPointerWrapper d);
    void reset();
    void update( float dt );

private:
    AuraAutopilot ap;
    
    PropertyNode status_node;
    PropertyNode ap_node;
    PropertyNode targets_node;
    PropertyNode tecs_node;
    PropertyNode task_node;
    PropertyNode pilot_node;
    PropertyNode flight_node;
    PropertyNode engine_node;
    PropertyNode route_node;
    PropertyNode active_node;
    PropertyNode home_node;
    PropertyNode circle_node;
    PropertyNode pos_node;

    void copy_pilot_inputs();
};


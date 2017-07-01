// Total Energy = Potential Energy (altitude) + Kinetic Energy (speed)
// E(t) = E(p) + E(k)
// E(t) = alt_m*kg*g + 0.5*vel^2

// Controlling total energy with throttle can be more stable because
// throttle changes can be coupled with pitch/speed/altitude changes
// in ways that are aircraft specific.  Minimizing throttle change can
// lead to more stable flight control.

#include "python/pyprops.hxx"
#include "include/globaldefs.h"

// input/output nodes
static pyPropertyNode pos_node;
static pyPropertyNode vel_node;
static pyPropertyNode filters_node;
static pyPropertyNode targets_node;

static bool tecs_inited = false;

static const float g = 9.81;

static double tecs_compute(double alt_m, double vel_mps, double mass) {
    double total_energy = mass * ( g * alt_m + 0.5 * vel_mps * vel_mps );
    return total_energy;
}


static void init_tecs() {
    pos_node = pyGetNode( "/position", true);
    vel_node = pyGetNode( "/velocity", true);
    filters_node = pyGetNode( "/filters", true);
    targets_node = pyGetNode( "/autopilot/targets", true );
    tecs_inited = true;
}


// compute the current total energy
void update_tecs() {
    if ( !tecs_inited ) {
        init_tecs();
    }

    double alt_m = pos_node.getDouble("altitude_agl_m");
    double vel_mps = vel_node.getDouble("airspeed_kt") * SG_KT_TO_MPS;
    double aircraft_kg = 2.5;
    
    double tecs = tecs_compute( alt_m, vel_mps, aircraft_kg );
    filters_node.setDouble("tecs", tecs);
}


// compute the target total energy
void update_tecs_target() {
    if ( !tecs_inited ) {
        init_tecs();
    }
    
    double alt_m = targets_node.getDouble("altitude_agl_ft") * SG_FEET_TO_METER;
    double vel_mps = targets_node.getDouble("airspeed_kt") * SG_KT_TO_MPS;
    double aircraft_kg = 2.5;

    double tecs = tecs_compute( alt_m, vel_mps, aircraft_kg );
    targets_node.setDouble("tecs", tecs);
}

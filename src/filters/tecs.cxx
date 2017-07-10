// Total Energy = Potential Energy (altitude) + Kinetic Energy (speed)
// E(t) = E(p) + E(k)
// E(t) = alt_m*kg*g + 0.5*vel^2

// Controlling total energy with throttle can be more stable because
// throttle changes can be coupled with pitch/speed/altitude changes
// in ways that are aircraft specific.  Minimizing throttle change can
// lead to more stable flight control.

// For most precise speed management, use kinetic energy error to
// drive a target pitch angle.  Using Kin_error - Pot_error (or some
// weighted difference) to drive pitch angle could control the
// distribution of energy between kinetic and potential, but could
// lead to overspeed in descents and underspeed in climbs.

#include "python/pyprops.hxx"
#include "include/globaldefs.h"

// input/output nodes
static pyPropertyNode pos_node;
static pyPropertyNode vel_node;
static pyPropertyNode specs_node;
static pyPropertyNode targets_node;
static pyPropertyNode tecs_node;
static pyPropertyNode tecs_config_node;

static bool tecs_inited = false;

static double mass_kg = 2.5;
static const float g = 9.81;


static void init_tecs() {
    pos_node = pyGetNode( "/position", true);
    vel_node = pyGetNode( "/velocity", true);
    specs_node = pyGetNode( "/config/specs", true);
    targets_node = pyGetNode( "/autopilot/targets", true);
    tecs_node = pyGetNode( "/filters/tecs", true);
    tecs_config_node = pyGetNode( "/config/TECS_controller", true);

    double m = specs_node.getDouble("mass_kg");
    if ( m > 0.01 ) {
        mass_kg = m;
    }

    tecs_inited = true;
}


// compute various energy metrics and errors
void update_tecs() {
    if ( !tecs_inited ) {
        init_tecs();
    }

    // Current energy
    double alt_m = pos_node.getDouble("altitude_agl_m");
    double vel_mps = vel_node.getDouble("airspeed_kt") * SG_KT_TO_MPS;

    double energy_pot = mass_kg * g * alt_m;
    double energy_kin = 0.5 * mass_kg * vel_mps * vel_mps;
    double energy_total = energy_pot + energy_kin;

    tecs_node.setDouble("energy_total", energy_total);
    tecs_node.setDouble("energy_pot", energy_pot);
    tecs_node.setDouble("energy_kin", energy_kin);

    // Target energy
    double target_alt_m = targets_node.getDouble("altitude_agl_ft") * SG_FEET_TO_METER;
    double target_vel_mps = targets_node.getDouble("airspeed_kt") * SG_KT_TO_MPS;

    double target_pot = mass_kg * g * target_alt_m;
    double target_kin = 0.5 * mass_kg * target_vel_mps * target_vel_mps;
    double target_total = target_pot + target_kin;

    tecs_node.setDouble("target_total", target_total);
    tecs_node.setDouble("target_pot", target_pot);
    tecs_node.setDouble("target_kin", target_kin);

    // Error metrics
    double error_pot = target_pot - energy_pot;
    double error_kin = target_kin - energy_kin;
    double error_total = error_pot + error_kin;
    
    tecs_node.setDouble("error_total", error_total);
    tecs_node.setDouble("error_pot", error_pot);
    tecs_node.setDouble("error_kin", error_kin);
    
    // compute min & max kinetic energy
    double min_kt = specs_node.getDouble("min_kt");
    if ( min_kt < 15 ) { min_kt = 15;}
    double min_mps = min_kt * SG_KT_TO_MPS;
    double min_kinetic = 0.5 * mass_kg * min_mps * min_mps;
    double min_error = min_kinetic - energy_kin;

    double max_kt = specs_node.getDouble("max_kt");
    if ( max_kt < 15 ) { max_kt = 2 * min_kt; }
    double max_mps = max_kt * SG_KT_TO_MPS;
    double max_kinetic = 0.5 * mass_kg * max_mps * max_mps;
    double max_error = max_kinetic - energy_kin;
    
    // Weighted kinetic + potential error
    double weight = tecs_node.getDouble("weight");
    if ( weight < 0.0 ) {
        weight = 0.0;
        tecs_node.setDouble("weight", weight);
    } else if ( weight > 1.0 ) {
        weight = 1.0;
        tecs_node.setDouble("weight", weight);
    }
    double error_blend =  (1.0 - weight) * error_kin - weight * error_pot;
    // printf("%.1f  %.1f  %.1f\n", min_error, error_blend, max_error);
    if ( error_blend < min_error ) { error_blend = min_error; }
    if ( error_blend > max_error ) { error_blend = max_error; }
    tecs_node.setDouble("error_blend", error_blend);
        
}

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

static const float g = 9.81;


static void init_tecs() {
    pos_node = pyGetNode( "/position", true);
    vel_node = pyGetNode( "/velocity", true);
    targets_node = pyGetNode( "/autopilot/targets", true);
    tecs_node = pyGetNode( "/autopilot/tecs", true);
    tecs_config_node = pyGetNode( "/config/autopilot/TECS", true);

    // force default weight values if the field is empty (so we can
    // differentiate "" from 0.0
    if ( ! tecs_config_node.hasChild("weight_tot") ) {
        tecs_config_node.setDouble("weight_tot", 1.0);
    }
    if ( ! tecs_config_node.hasChild("weight_bal") ) {
        tecs_config_node.setDouble("weight_bal", 1.0);
    }

    tecs_inited = true;
}


// compute various energy metrics and errors
void update_tecs() {
    if ( !tecs_inited ) {
        init_tecs();
    }

    double mass_kg = tecs_config_node.getDouble("mass_kg");
    if ( mass_kg < 0.01 ) { mass_kg = 2.5; }
    
    // total weight factor
    double wt = tecs_config_node.getDouble("weight_tot");
    if ( wt < 0.0 ) {
        wt = 0.0;
        tecs_config_node.setDouble("weight_tot", wt);
    } else if ( wt > 2.0 ) {
        wt = 2.0;
        tecs_config_node.setDouble("weight_tot", wt);
    }
    
    // balance weight factor
    double wb = tecs_config_node.getDouble("weight_bal");
    if ( wb < 0.0 ) {
        wb = 0.0;
        tecs_config_node.setDouble("weight_bal", wb);
    } else if ( wb > 2.0 ) {
        wb = 2.0;
        tecs_config_node.setDouble("weight_bal", wb);
    }

    // Current energy
    double alt_m = pos_node.getDouble("altitude_agl_m");
    double vel_mps = vel_node.getDouble("airspeed_smoothed_kt") * SG_KT_TO_MPS;

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
    double error_total = wt * error_pot + (2.0 - wt) * error_kin;

    // Compute min & max kinetic energy
    double min_kt = tecs_config_node.getDouble("min_kt");
    if ( min_kt < 15 ) { min_kt = 15;}
    double min_mps = min_kt * SG_KT_TO_MPS;
    double min_kinetic = 0.5 * mass_kg * min_mps * min_mps;
    double min_error = min_kinetic - energy_kin;

    double max_kt = tecs_config_node.getDouble("max_kt");
    if ( max_kt < 15 ) { max_kt = 2 * min_kt; }
    double max_mps = max_kt * SG_KT_TO_MPS;
    double max_kinetic = 0.5 * mass_kg * max_mps * max_mps;
    double max_error = max_kinetic - energy_kin;

    // Guard against overspeed situations (presumably we've hit our
    // max pitch up limit and have enough power to accelerate past our
    // top speed.)  Is there a way to limit error_total using energy
    // error metrics?  Or do we just live with the potential of
    // overspeed climbs with over powered aircraft?
    if ( max_error < 0 ) {
        // we are too fast
        // printf("overspeed: %.1f %.1f %.1f\n", error_total, error_kin, error_pot);
    }     
        
    tecs_node.setDouble("error_total", error_total);
    tecs_node.setDouble("error_pot", error_pot);
    tecs_node.setDouble("error_kin", error_kin);
    
    double error_diff =  (2.0 - wb) * error_kin - wb * error_pot;
    // printf("%.1f  %.1f  %.1f\n", min_error, error_diff, max_error);

    // enforce speed limits (in energy error space)
    if ( error_diff < min_error ) { error_diff = min_error; }
    if ( error_diff > max_error ) { error_diff = max_error; }
    
    tecs_node.setDouble("error_diff", error_diff);
}

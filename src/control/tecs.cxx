// Total Energy = Potential Energy (altitude) + Kinetic Energy (speed)
// PE = alt_m * kg * g
// KE = 0.5 * kg * vel^2

// This system controls altitude and airspeed with throttle and pitch
// angle.  Throttle and pitch angle are used as analogs for power and
// flight path angle.

// We observe: (1) Throttle (thrust, power) has a fairly direct affect
// on the total energy of the system.  (2) Conservation of energy
// suggests we can trade PE for KE and visa versa without affecting
// the total energy of a system.  (3) In an otherwise stable system,
// for small changes and small time periods, theta (pitch angle)
// changes can be used to trade PE vs KE.

// This system computes potential energy *error* and kinetic energy
// *error*.  Throttle is used to drive the sum of these errors to
// zero.  Pitch angle is used to drive the difference of these errors
// to zero.

// Because of the way this system is formulated, we can establish a
// max/min speed range.  These limits can be computed in terms of
// energy error and used to limit the energy error sum and energy
// error difference.  Thus (outside of sensor noise, turbulence, and
// PID overshoot) the system will never command a combination of
// throttle positition and pitch angle that will force those limits to
// be exceeded.  This produces a well behaved system that will never
// underspeed in a climb or overspeed in a dive.

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
    // if ( ! tecs_config_node.hasChild("weight_tot") ) {
    //     tecs_config_node.setDouble("weight_tot", 1.0);
    // }
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

    // Energy error
    double error_pot = target_pot - energy_pot;
    double error_kin = target_kin - energy_kin;
    tecs_node.setDouble("error_pot", error_pot);
    tecs_node.setDouble("error_kin", error_kin);
    
    // Compute min & max kinetic energy allowed (based on configured
    // operational speed range)
    double min_kt = tecs_config_node.getDouble("min_kt");
    if ( min_kt < 15 ) { min_kt = 15;}
    double min_mps = min_kt * SG_KT_TO_MPS;
    double min_kinetic = 0.5 * mass_kg * min_mps * min_mps;

    double max_kt = tecs_config_node.getDouble("max_kt");
    if ( max_kt < 15 ) { max_kt = 2 * min_kt; }
    double max_mps = max_kt * SG_KT_TO_MPS;
    double max_kinetic = 0.5 * mass_kg * max_mps * max_mps;

    // Set min & max kinetic energy errors allowed (prevent us from
    // exceeding allowed kinetic energy range)
    double min_error = min_kinetic - energy_kin;
    double max_error = max_kinetic - energy_kin;

    // if min_error > 0: we are underspeed
    // if max_error < 0: we are overspeed
    
    // total energy error and (weighted) energy balance
    double error_total = error_pot + error_kin;
    double error_diff =  (2.0 - wb) * error_kin - wb * error_pot;
    
    // Note: we can clamp the total error and error balance (in energy
    // error space) to establish speed limits.  This prevents a large
    // altitude error for over or underspeeding the aircraft.  Also,
    // prevents over speeding in a climb if max pitch angle is
    // saturated.
    
    // clamp error_diff to kinetic error range
    if ( error_diff < min_error ) { error_diff = min_error; }
    if ( error_diff > max_error ) { error_diff = max_error; }

    // clamp max total error to avoid overspeed in a climb if max
    // pitch angle is saturated.
    if ( error_total > max_error ) { error_total = max_error; }

    // publish the final values
    tecs_node.setDouble("error_total", error_total);
    tecs_node.setDouble("error_diff", error_diff);
}

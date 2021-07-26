#include <props2.h>

#include "include/globaldefs.h"
#include "util/lowpass.h"

static PropertyNode filter_node;
static PropertyNode pos_filter_node;
static PropertyNode task_node;

// initial values are the 'time factor'
static LowPassFilter ground_alt_filt( 30.0 );

static bool ground_alt_calibrated = false;

// initialize ground estimator variables
void init_ground() {
    filter_node = PropertyNode("/filters/filter", true);
    pos_filter_node = PropertyNode("/position/filter", true);
    task_node = PropertyNode("/task", true);
}

void update_ground(double dt) {
    // determine ground reference altitude.  Average filter altitude
    // over the most recent 30 seconds that we are !is_airborne
    if ( !ground_alt_calibrated ) {
	ground_alt_calibrated = true;
	ground_alt_filt.init( filter_node.getFloat("altitude_m") );
    }

    if ( ! task_node.getBool("is_airborne") ) {
	// ground reference altitude averaged current altitude over
	// first 30 seconds while on the ground
	ground_alt_filt.update( filter_node.getFloat("altitude_m"), dt );
	pos_filter_node.setFloat( "altitude_ground_m",
                                  ground_alt_filt.get_value() );
    }

    float agl_m = filter_node.getFloat( "altitude_m" )
	- ground_alt_filt.get_value();
    pos_filter_node.setFloat( "altitude_agl_m", agl_m );
    pos_filter_node.setFloat( "altitude_agl_ft", agl_m * SG_METER_TO_FEET );
}

#include <pyprops.h>

#include "include/globaldefs.h"
#include "util/lowpass.h"

#include "wind.h"

static pyPropertyNode airdata_node;
static pyPropertyNode filter_node;
static pyPropertyNode orient_node;
static pyPropertyNode task_node;
static pyPropertyNode vel_node;
static pyPropertyNode wind_node;

static LowPassFilter we_filt( 60.0 );
static LowPassFilter wn_filt( 60.0 );
static LowPassFilter pitot_scale_filt( 30.0 );

// initialize wind estimator variables
void init_wind() {
    airdata_node = pyGetNode("/sensors/airdata", true);
    filter_node = pyGetNode("/filters/filter", true);
    orient_node = pyGetNode("/orientation", true);
    task_node = pyGetNode("/task", true);
    vel_node = pyGetNode("/velocity", true);
    wind_node = pyGetNode("/filters/wind", true);
    
    wind_node.setDouble( "pitot_scale_factor", 1.0 );

    pitot_scale_filt.init(1.0);
}


// onboard wind estimate (requires airspeed, true heading, and ground
// velocity vector)
void update_wind( double dt ) {
    double airspeed_kt = airdata_node.getDouble("airspeed_kt");
    if ( ! task_node.getBool("is_airborne") ) {
	// System predicts we are not flying.  The wind estimation
	// code only works in flight.
	return;
    }

    double psi = SGD_PI_2
	- orient_node.getDouble("heading_deg") * SG_DEGREES_TO_RADIANS;
    double pitot_scale = pitot_scale_filt.get_value();
    double ue = cos(psi) * (airspeed_kt * pitot_scale * SG_KT_TO_MPS);
    double un = sin(psi) * (airspeed_kt * pitot_scale * SG_KT_TO_MPS);
    double we = ue - filter_node.getDouble("ve_ms");
    double wn = un - filter_node.getDouble("vn_ms");

    //static double filt_we = 0.0, filt_wn = 0.0;
    //filt_we = 0.9998 * filt_we + 0.0002 * we;
    //filt_wn = 0.9998 * filt_wn + 0.0002 * wn;
    we_filt.update(we, dt);
    wn_filt.update(wn, dt);

    double we_filt_val = we_filt.get_value();
    double wn_filt_val = wn_filt.get_value();
    
    double wind_deg = 90
	- atan2( wn_filt_val, we_filt_val ) * SGD_RADIANS_TO_DEGREES;
    if ( wind_deg < 0 ) { wind_deg += 360.0; }
    double wind_speed_kt = sqrt( we_filt_val*we_filt_val
				 + wn_filt_val*wn_filt_val ) * SG_MPS_TO_KT;

    wind_node.setDouble( "wind_speed_kt", wind_speed_kt );
    wind_node.setDouble( "wind_dir_deg", wind_deg );
    wind_node.setDouble( "wind_east_mps", we_filt_val );
    wind_node.setDouble( "wind_north_mps", wn_filt_val );

    // estimate pitot tube bias
    double true_e = we_filt_val + filter_node.getDouble("ve_ms");
    double true_n = wn_filt_val + filter_node.getDouble("vn_ms");

    double true_deg = 90 - atan2( true_n, true_e ) * SGD_RADIANS_TO_DEGREES;
    if ( true_deg < 0 ) { true_deg += 360.0; }
    double true_speed_kt = sqrt( true_e*true_e + true_n*true_n ) * SG_MPS_TO_KT;

    wind_node.setDouble( "true_airspeed_kt", true_speed_kt );
    wind_node.setDouble( "true_heading_deg", true_deg );
    wind_node.setDouble( "true_airspeed_east_mps", true_e );
    wind_node.setDouble( "true_airspeed_north_mps", true_n );

    double ps = 1.0;
    if ( airspeed_kt > 1.0 ) {
	ps = true_speed_kt / airspeed_kt;
	// don't let the scale factor exceed some reasonable limits
	if ( ps < 0.75 ) { ps = 0.75;	}
	if ( ps > 1.25 ) { ps = 1.25; }
    }

    pitot_scale_filt.update(ps, dt);
    wind_node.setDouble( "pitot_scale_factor", pitot_scale_filt.get_value() );

    // if ( display_on ) {
    //   printf("true: %.2f kt  %.1f deg (scale = %.4f)\n", true_speed_kt, true_deg, pitot_scale_filt);
    // }

    // now estimate ground speed/track based on airdata and wind estimate
    double ve_est = ue - we_filt_val;
    double vn_est = un - wn_filt_val;
    double groundtrack_est_deg = 90 - atan2( vn_est, ve_est ) * SGD_RADIANS_TO_DEGREES;
    if ( groundtrack_est_deg < 0 ) { groundtrack_est_deg += 360.0; }
    double groundspeed_est_ms = sqrt( ve_est*ve_est + vn_est*vn_est );
    // double groundspeed_est_kt = groundspeed_est_ms * SG_MPS_TO_KT;
    vel_node.setDouble( "groundspeed_est_ms", groundspeed_est_ms );
    orient_node.setDouble( "groundtrack_est_deg", groundtrack_est_deg );
}

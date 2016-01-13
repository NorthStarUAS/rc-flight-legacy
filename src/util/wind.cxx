#include "python/pyprops.hxx"

#include <cmath>

#include "include/globaldefs.h"

#include "wind.hxx"


// Given a wind speed estimate, true airspeed estimate, wind direction
// estimate, and a desired course to fly, then compute a true heading to
// fly to achieve the desired course in the given wind.  Also compute the
// estimated ground speed.

void wind_course( double ws_kt, double tas_kt, double wd_deg, double crs_deg,
		  double *hd_deg, double *gs_kt)
{
    // from williams.best.vwh.net/avform.htm (aviation formulas)
    double wd = wd_deg * SGD_DEGREES_TO_RADIANS;
    double crs = crs_deg * SGD_DEGREES_TO_RADIANS;
    double hd = 0.0;
    *gs_kt = 0.0;
    *hd_deg = 0.0;
    if ( tas_kt > 0.1 ) {
	// printf("ws=%.1f tas=%.1f wd=%.1f crs=%.1f\n", ws, tas, wd, crs);
	double swc = (ws_kt/tas_kt)*sin(wd-crs);
	// printf("swc=%.2f\n", swc);
	if ( fabs(swc) > 1.0 ) {
	    // course cannot be flown, wind too strong
	    // point nose into estimated wind and "kite" as best we can
	    hd = wd + SGD_PI;
	    if ( hd > SGD_2PI ) { hd -= SGD_2PI; }
	} else {
	    hd = crs + asin(swc);
	    if ( hd < 0.0 ) { hd += SGD_2PI; }
	    if ( hd > SGD_2PI ) { hd -= SGD_2PI; }
	    *gs_kt = tas_kt * sqrt(1-swc*swc) - ws_kt * cos(wd - crs);
	}
	*hd_deg = hd * SGD_RADIANS_TO_DEGREES;
    }
    // if ( display_on ) {
    //   printf("af: hd=%.1f gs=%.1f\n", hd * SGD_RADIANS_TO_DEGREES, gs);
    // }
}


// Given wind speed and direction, true airspeed (pulled directly from
// the property tree), as well as a current ground course, and a
// target ground course, compute the estimated true heading (psi,
// aircraft body heading) difference that will take us from the
// current ground course to the target ground course.

double wind_heading_diff( double current_crs_deg, double target_crs_deg ) {
    // wind estimates
    static pyPropertyNode wind_node = pyGetNode("/filters/wind-est", true);

    // static SGPropertyNode *wind_speed_kt
    // 	= pyGetNode("/filters/wind-est/wind-speed-kt", true);
    // static SGPropertyNode *wind_dir_deg
    // 	= pyGetNode("/filters/wind-est/wind-dir-deg", true);
    // static SGPropertyNode *true_airspeed_kt
    // 	= pyGetNode("/filters/wind-est/true-airspeed-kt", true);
 
    double gs_kt = 0.0;

    double est_cur_hdg_deg = 0.0;
    wind_course( wind_node.getDouble("wind_speed_kt"),
		 wind_node.getDouble("true_airspeed_kt"),
		 wind_node.getDouble("wind_dir_deg"),
		 current_crs_deg,
		 &est_cur_hdg_deg, &gs_kt );

    double est_nav_hdg_deg = 0.0;
    wind_course( wind_node.getDouble("wind_speed_kt"),
		 wind_node.getDouble("true_airspeed_kt"),
		 wind_node.getDouble("wind_dir_deg"),
		 target_crs_deg,
		 &est_nav_hdg_deg, &gs_kt );

    double hdg_diff = est_nav_hdg_deg - est_cur_hdg_deg;
    if ( hdg_diff < -180.0 ) { hdg_diff += 360.0; }
    if ( hdg_diff > 180.0 ) { hdg_diff -= 360.0; }

    return hdg_diff;
}

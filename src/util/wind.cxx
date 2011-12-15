#include <cmath>

#include "include/globaldefs.h"
#include "props/props.hxx"

#include "wind.hxx"


// Given a wind speed estimate, true airspeed estimate, wind direction
// estimate, and a desired course to fly, then compute a true heading to
// fly to achieve the desired course in the given wind.  Also compute the
// resulting ground speed.

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

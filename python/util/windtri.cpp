#include <math.h>

#include "windtri.h"

static const double pi = 3.141592653589793;
static const double d2r = pi / 180.0;
static const double r2d = 180.0 / pi;
static const double m2pi = pi * 2.0;

// Given a wind speed estimate, true airspeed estimate, wind direction
// estimate, and a desired course to fly, then compute a true heading to
// fly to achieve the desired course in the given wind.  Also compute the
// estimated ground speed.

py::tuple wind_course( double ws_kt, double tas_kt, double wd_deg,
                       double crs_deg )
{
    // from williams.best.vwh.net/avform.htm (aviation formulas)
    double wd = wd_deg * d2r;
    double crs = crs_deg * d2r;
    double hd = 0.0;
    double gs_kt = 0.0;
    double hd_deg = 0.0;
    if ( tas_kt > 0.1 ) {
	// printf("ws=%.1f tas=%.1f wd=%.1f crs=%.1f\n", ws_kt, tas_kt, wd_deg, crs_deg);
        // printf("wd=%.4f crs=%.4f\n", wd, crs);
        // printf("wd-crs=%.4f\n", wd-crs);
        // printf("sin(wd-crs)=%.4f\n", sin(wd-crs));
	double swc = (ws_kt/tas_kt)*sin(wd-crs);
	// printf("swc=%.4f\n", swc);
	if ( fabs(swc) > 1.0 ) {
	    // course cannot be flown, wind too strong
	    // point nose into estimated wind and "kite" as best we can
	    hd = wd + pi;
	    if ( hd > m2pi ) { hd -= m2pi; }
	} else {
	    hd = crs + asin(swc);
	    if ( hd < 0.0 ) { hd += m2pi; }
	    if ( hd > m2pi ) { hd -= m2pi; }
	    gs_kt = tas_kt * sqrt(1-swc*swc) - ws_kt * cos(wd - crs);
	}
	hd_deg = hd * r2d;
    }
    // if ( display_on ) {
    //   printf("af: hd=%.1f gs=%.1f\n", hd * SGD_RADIANS_TO_DEGREES, gs);
    // }
    return py::make_tuple(hd_deg, gs_kt);
}


#ifdef HAVE_PYBIND11
  PYBIND11_PLUGIN(windtri) {
      py::module m("windtri", "wind triangle calcs for python");
      m.def("wind_course", &wind_course);
      return m.ptr();
  }
#endif // HAVE_PYBIND11

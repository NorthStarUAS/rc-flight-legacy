// Given a wind speed estimate, true airspeed estimate, wind direction
// estimate, and a desired course to fly, then compute a true heading to
// fly to achieve the desired course in the given wind.  Also compute the
// estimated ground speed.

void wind_course( double ws_kt, double tas_kt, double wd_deg, double crs_deg,
		  double *hd_deg, double *gs_kt);


// Given wind speed and direction, true airspeed (pulled directly from
// the property tree), as well as a current ground course, and a
// target ground course, compute the estimated true heading (psi,
// aircraft body heading) difference that will take us from the
// current ground course to the target ground course.

double wind_heading_diff( double current_crs_deg, double target_crs_deg );

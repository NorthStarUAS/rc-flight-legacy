// Given a wind speed estimate, true airspeed estimate, wind direction
// estimate, and a desired course to fly, then compute a true heading to
// fly to achieve the desired course in the given wind.  Also compute the
// resulting ground speed.

void wind_course( double ws_kt, double tas_kt, double wd_deg, double crs_deg,
		  double *hd_deg, double *gs_kt);

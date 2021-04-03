#pragma once

#include <pybind11/pybind11.h>
namespace py = pybind11;

// Given a wind speed estimate, true airspeed estimate, wind direction
// estimate, and a desired course to fly, then compute a true heading to
// fly to achieve the desired course in the given wind.  Also compute the
// estimated ground speed.

py::tuple wind_course( double ws_kt, double tas_kt, double wd_deg,
                       double crs_deg );

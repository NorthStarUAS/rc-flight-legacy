#!/usr/bin/env python3

# script to build and install all the required C++ modules as python
# extensions.  Depends on pybind11 and rc-props as the connecting glue
# between python and C++.

from setuptools import setup, Extension

setup(
    name="rcUAS",
    version="1.0",
    description="Rice Creek UAS",
    author="Curtis L. Olson",
    author_email="curtolson@flightgear.org",
    url="https://github.com/RiceCreekUAS",
    ext_modules=[
        Extension("rcUAS.actuator_mgr",
                  define_macros=[("HAVE_PYBIND11", "1")],
                  sources=[
                      "src/control/actuators.cpp",
                      "src/util/timing.cpp"
                  ],
                  depends=[
                      "src/control/actuators.h",
                      "src/util/timing.h"
                  ],
                  include_dirs=["src"],
                  extra_objects=["/usr/local/lib/libprops2.a"]
                  ),
        Extension("rcUAS.control_mgr",
                  define_macros=[("HAVE_PYBIND11", "1")],
                  sources=[
                      "src/control/ap.cpp",
                      "src/control/cas.cpp",
                      "src/control/control.cpp",
                      "src/control/dig_filter.cpp",
                      "src/control/dtss.cpp",
                      "src/control/pid.cpp",
                      "src/control/pid_vel.cpp",
                      "src/control/predictor.cpp",
                      "src/control/summer.cpp",
                      "src/control/tecs.cpp",
                      "src/util/timing.cpp"
                  ],
                  depends=[
                      "src/control/ap.h",
                      "src/control/cas.h",
                      "src/control/control.h",
                      "src/control/dig_filter.h",
                      "src/control/dtss.h",
                      "src/control/pid.h",
                      "src/control/pid_vel.h",
                      "src/control/predictor.h",
                      "src/control/summer.h",
                      "src/control/tecs.h",
                      "src/util/timing.h"
                  ],
                  include_dirs=["src"],
                  extra_objects=["/usr/local/lib/libprops2.a"]
                  ),
        Extension("rcUAS.driver_mgr",
                  define_macros=[("HAVE_PYBIND11", "1")],
                  sources=[
                      "src/drivers/Aura4/Aura4.cpp",
                      "src/drivers/rcfmu/rcfmu.cpp",
                      "src/drivers/rcfmu/serial_link2.cpp",
                      "src/drivers/driver_mgr.cpp",
                      "src/drivers/fgfs.cpp",
                      "src/drivers/gps_gpsd.cpp",
                      "src/drivers/lightware.cpp",
                      "src/drivers/maestro.cpp",
                      "src/drivers/ublox6.cpp",
                      "src/drivers/ublox8.cpp",
                      "src/drivers/ublox9.cpp",
                      "src/filters/nav_common/coremag.c",
                      "src/filters/nav_common/nav_functions.cpp",
                      "src/util/butter.cpp",
                      "src/util/geodesy.cpp",
                      "src/util/linearfit.cpp",
                      "src/util/lowpass.cpp",
                      "src/util/netSocket.cpp",
                      "src/util/props_helper.cpp",
                      "src/util/ratelimiter.cpp",
                      "src/util/serial_link.cpp",
                      "src/util/sg_path.cpp",
                      "src/util/strutils.cpp",
                      "src/util/timing.cpp"
                  ],
                  depends=[
                      "src/drivers/Aura4/aura4_messages.h",
                      "src/drivers/Aura4/Aura4.h",
                      "src/drivers/rcfmu/rcfmu.h",
                      "src/drivers/rcfmu/ns_messages.h",
                      "src/drivers/rcfmu/serial_link2.h",
                      "src/drivers/driver.h",
                      "src/drivers/driver_mgr.h",
                      "src/drivers/fgfs.h",
                      "src/drivers/gps_gpsd.h",
                      "src/drivers/lightware.h",
                      "src/drivers/maestro.h",
                      "src/drivers/ublox6.h",
                      "src/drivers/ublox8.h",
                      "src/drivers/ublox9.h",
                      "src/filters/nav_common/coremag.h",
                      "src/filters/nav_common/nav_functions.h",
                      "src/util/butter.h",
                      "src/util/geodesy.h",
                      "src/util/linearfit.h",
                      "src/util/lowpass.h",
                      "src/util/netSocket.h",
                      "src/util/props_helper.h",
                      "src/util/ratelimiter.h",
                      "src/util/serial_link.h",
                      "src/util/sg_path.h",
                      "src/util/strutils.h",
                      "src/util/timing.h"
                  ],
                  include_dirs=["src"],
                  extra_objects=["/usr/local/lib/libprops2.a"]
                  ),
        Extension("rcUAS.gps_helper",
                  define_macros=[("HAVE_PYBIND11", "1")],
                  sources=[
                      "src/drivers/gps.cpp",
                      "src/filters/nav_common/coremag.c",
                      "src/util/timing.cpp"
                  ],
                  depends=[
                      "src/drivers/gps.h",
                      "src/filters/nav_common/coremag.h",
                      "src/util/timing.h"
                  ],
                  include_dirs=["src"],
                  extra_objects=["/usr/local/lib/libprops2.a"]
                  ),
        Extension("rcUAS.filter_mgr",
                  define_macros=[("HAVE_PYBIND11", "1")],
                  sources=[
                      "src/filters/filter_mgr.cpp",
                      "src/filters/wind.cpp",
                      "src/filters/nav_ekf15/aura_interface.cpp",
                      "src/filters/nav_ekf15/EKF_15state.cpp",
                      "src/filters/nav_ekf15_mag/aura_interface.cpp",
                      "src/filters/nav_ekf15_mag/EKF_15state.cpp",
                      "src/filters/nav_common/coremag.c",
                      "src/filters/nav_common/nav_functions.cpp",
                      "src/util/lowpass.cpp",
                      "src/util/props_helper.cpp"
                  ],
                  depends=[
                      "src/filters/filter_mgr.h",
                      "src/filters/wind.h",
                      "src/filters/nav_ekf15/aura_interface.h",
                      "src/filters/nav_ekf15/EKF_15state.h",
                      "src/filters/nav_ekf15_mag/aura_interface.h",
                      "src/filters/nav_ekf15_mag/EKF_15state.h",
                      "src/filters/nav_common/coremag.h",
                      "src/filters/nav_common/nav_functions.h",
                      "src/util/lowpass.h",
                      "src/util/props_helper.h"
                  ],
                  include_dirs=["src"],
                  extra_objects=["/usr/local/lib/libprops2.a"]
                  ),
        Extension("rcUAS.wgs84",
                  define_macros=[("HAVE_PYBIND11", "1")],
                  sources=["src/util/wgs84.cpp"],
                  depends=["src/util/wgs84.h"]
                  ),
        Extension("rcUAS.windtri",
                  define_macros=[("HAVE_PYBIND11", "1")],
                  sources=["src/util/windtri.cpp"],
                  depends=["src/util/windtri.h"]
                  )
    ]
)

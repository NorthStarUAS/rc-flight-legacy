#!/usr/bin/python3

# a quick first hack/test

from setuptools import setup, Extension

setup(
    name="rcUAS",
    version="1.0",
    description="Rice Creek UAS",
    author="Curtis L. Olson",
    author_email="curtolson@flightgear.org",
    url="https://github.com/RiceCreekUAS",
    ext_modules=[
        Extension("rcUAS.driver_mgr",
                  define_macros=[("HAVE_PYBIND11", "1")],
                  sources=[
                      "src/drivers/Aura4/Aura4.cpp",
                      "src/drivers/driver_mgr.cpp",
                      "src/drivers/fgfs.cpp",
                      "src/drivers/gps_gpsd.cpp",
                      "src/drivers/lightware.cpp",
                      "src/drivers/maestro.cpp",
                      "src/drivers/raw_sat.cpp",
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
                      "src/util/serial_link.cpp",
                      "src/util/sg_path.cpp",
                      "src/util/strutils.cpp",
                      "src/util/timing.cpp"
                  ],
                  depends=[
                      "src/drivers/Aura4/aura4_messages.h",
                      "src/drivers/Aura4/Aura4.h",
                      "src/drivers/driver.h",
                      "src/drivers/driver_mgr.h",
                      "src/drivers/fgfs.h",
                      "src/drivers/gps_gpsd.h",
                      "src/drivers/lightware.h",
                      "src/drivers/maestro.h",
                      "src/drivers/raw_sat.h",
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
                      "src/util/serial_link.h",
                      "src/util/sg_path.h",
                      "src/util/strutils.h",
                      "src/util/timing.h"
                  ],
                  include_dirs=["src"],
                  extra_objects=["/usr/local/lib/libpyprops.a"]
                  ),
        Extension("rcUAS.airdata_helper",
                  define_macros=[("HAVE_PYBIND11", "1")],
                  sources=[
                      "src/drivers/airdata.cpp",
                      "src/util/lowpass.cpp"
                  ],
                  depends=[
                      "src/drivers/airdata.h",
                      "src/util/lowpass.h"
                  ],
                  include_dirs=["src"],
                  extra_objects=["/usr/local/lib/libpyprops.a"]
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
                  extra_objects=["/usr/local/lib/libpyprops.a"]
                  ),
        Extension("rcUAS.wgs84",
                  define_macros=[("HAVE_PYBIND11", "1")],
                  sources=["python/util/wgs84.cpp"],
                  depends=["python/util/wgs84.h"]
                  ),
        Extension("rcUAS.windtri",
                  define_macros=[("HAVE_PYBIND11", "1")],
                  sources=["python/util/windtri.cpp"],
                  depends=["python/util/windtri.h"]
                  )
    ]
)

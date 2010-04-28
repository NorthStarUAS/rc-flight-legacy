EXTRA_DIST = config.xml

if ENABLE_MNAV_FILTER
MNAV_LIBS = $(top_builddir)/src/filters/mnav/libadns_mnav.a
else
MNAV_LIBS =
endif

bin_PROGRAMS = ugear

ugear_SOURCES = \
	globals.cxx globals.hxx \
        imu_mgr.cxx imu_mgr.hxx \
	ugear_gumstix.cpp

ugear_LDADD = \
	$(top_builddir)/src/actuators/libactuators.a \
	$(top_builddir)/src/filters/libfilters.a \
	$(top_builddir)/src/filters/curt/libadns_curt.a \
	$(MNAV_LIBS) \
	$(top_builddir)/src/health/libhealth.a \
	$(top_builddir)/src/math/libmath.a \
	$(top_builddir)/src/sensors/libsensors.a \
	$(top_builddir)/src/comms/libcomms.a \
	$(top_builddir)/src/control/libcontrol.a \
	$(top_builddir)/src/props/libsgprops.a \
	$(top_builddir)/src/util/libutil.a \
	$(top_builddir)/src/xml/libsgxml.a

INCLUDES = -I$(top_srcdir)/src

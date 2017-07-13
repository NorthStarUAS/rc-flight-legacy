bin_PROGRAMS = aura

aura_SOURCES = \
	aura.cxx

aura_LDADD = \
	../actuators/libactuators.a \
	../filters/libfilters.a \
	../filters/nav_eigen/libnav_eigen.a \
	../filters/nav_eigen_mag/libnav_eigen_mag.a \
	../filters/umngnss_quat/libumngnss_quat.a \
	../health/libhealth.a \
	../payload/libpayload.a \
	../sensors/libsensors.a \
	../init/libinit.a \
	../control/libcontrol.a \
	../comms/libcomms.a \
	../math/libmath.a \
	../util/libutil.a \
	../python/libpyprops.a \
	-lpython2.7

AM_CPPFLAGS = -I$(top_srcdir)/src -I$(top_builddir)/src

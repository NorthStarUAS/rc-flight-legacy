bin_PROGRAMS = aura

aura_SOURCES = \
	aura.cxx

aura_LDADD = \
	../actuators/libactuators.a \
	../filters/libfilters.a \
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
	@PYTHON_LIBS@

AM_CPPFLAGS = \
	-I$(top_srcdir)/src \
	-I$(top_builddir)/src \
	@PYTHON_INCLUDES@

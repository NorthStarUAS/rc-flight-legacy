bin_PROGRAMS = aura

aura_SOURCES = \
	aura.cpp

aura_LDADD = \
	../actuators/libactuators.a \
	../filters/libfilters.a \
	../filters/nav_ekf15/libnav_ekf15.a \
	../filters/nav_ekf15_mag/libnav_ekf15_mag.a \
	../filters/nav_common/libnav_common.a \
	../health/libhealth.a \
	../payload/libpayload.a \
	../sensors/libsensors.a \
	../sensors/Aura3/libAura3.a \
	../init/libinit.a \
	../control/libcontrol.a \
	../comms/libcomms.a \
	../util/libutil.a \
	$(PYTHON_LIBS)

AM_CPPFLAGS = $(PYTHON_INCLUDES) -I$(top_srcdir)/src -I$(top_builddir)/src

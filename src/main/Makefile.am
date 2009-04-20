# For StarGate compile uncomment the following lines:
# ugear_LDFLAGS = -nodefaultlibs
# ugear_MORELIBS = -lm -lrt -lgcc -lc

# For GumStix compile uncomment the following lines instead:
ugear_LDFLAGS =
ugear_MORELIBS =

bin_PROGRAMS = ugear ugear_timer decoder

ugear_SOURCES = \
	ugear.cpp

ugear_LDADD = \
	$(top_builddir)/src/actuators/libactuators.a \
	$(top_builddir)/src/comms/libcomms.a \
	$(top_builddir)/src/health/libhealth.a \
	$(top_builddir)/src/sensors/libsensors.a \
	$(top_builddir)/src/navigation/libnavigation.a \
	$(top_builddir)/src/control/libcontrol.a \
	$(top_builddir)/src/props/libsgprops.a \
	$(top_builddir)/src/util/libutil.a \
	$(top_builddir)/src/xml/libsgxml.a \
	$(ugear_MORELIBS)

ugear_timer_SOURCES = \
	ugear_timer.cpp

ugear_timer_LDADD = \
	$(top_builddir)/src/actuators/libactuators.a \
	$(top_builddir)/src/comms/libcomms.a \
	$(top_builddir)/src/health/libhealth.a \
	$(top_builddir)/src/sensors/libsensors.a \
	$(top_builddir)/src/navigation/libnavigation.a \
	$(top_builddir)/src/control/libcontrol.a \
	$(top_builddir)/src/props/libsgprops.a \
	$(top_builddir)/src/util/libutil.a \
	$(top_builddir)/src/xml/libsgxml.a \
	$(ugear_MORELIBS)

decoder_SOURCES = decoder.c
decoder_LDADD =

INCLUDES = -I$(top_srcdir)/src

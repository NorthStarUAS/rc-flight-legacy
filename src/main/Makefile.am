bin_PROGRAMS = ugear decoder

ugear_SOURCES = \
	ugear.cpp

ugear_LDFLAGS = -nodefaultlibs
ugear_LDADD = \
	$(top_builddir)/src/comms/libcomms.a \
	$(top_builddir)/src/control/libcontrol.a \
	$(top_builddir)/src/health/libhealth.a \
	$(top_builddir)/src/navigation/libnavigation.a \
	$(top_builddir)/src/util/libutil.a \
	-lm -lgcc -lc

decoder_SOURCES = decoder.c
decoder_LDADD =

INCLUDES = -I$(top_srcdir)/src

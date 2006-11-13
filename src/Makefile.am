SUBDIRS = comms control include navigation

bin_PROGRAMS = avionics decoder

avionics_SOURCES = \
	globaldefs.h \
	main.cpp \
	matrix.c matrix.h \
	navfunc.cpp navfunc.h \
	timing.cpp timing.h \
	util.cpp util.h

decoder_SOURCES = decoder.c
decoder_LDADD =

avionics_LDFLAGS = -nodefaultlibs
avionics_LDADD = \
	$(top_builddir)/src/comms/libcomms.a \
	$(top_builddir)/src/control/libcontrol.a \
	$(top_builddir)/src/navigation/libnavigation.a \
	-lm -lgcc -lc

INCLUDES = -I$(top_srcdir)/src/comms -I$(top_srcdir)/src/navigation

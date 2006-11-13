SUBDIRS = navigation

bin_PROGRAMS = avionics decoder

avionics_SOURCES = \
	console_link.cpp console_link.h \
	control.cpp control.h \
	globaldefs.h \
	groundstation.cpp groundstation.h \
	logging.cpp logging.h \
	main.cpp \
	matrix.c matrix.h \
	navfunc.cpp navfunc.h \
	serial.cpp serial.h \
	timing.cpp timing.h \
	uplink.h uplink.cpp \
	util.cpp util.h

decoder_SOURCES = decoder.c
decoder_LDADD =

avionics_LDFLAGS = -nodefaultlibs
avionics_LDADD = \
	$(top_builddir)/src/navigation/libnavigation.a \
	-lm -lgcc -lc


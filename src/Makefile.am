bin_PROGRAMS = avionics decoder

avionics_SOURCES = \
	ahrs.cpp ahrs.h \
	control.cpp control.h \
	globaldefs.h \
	groundstation.cpp groundstation.h \
	imugps.cpp imugps.h \
	logging.cpp logging.h \
	main.cpp \
	matrix.c matrix.h \
	navfunc.cpp navfunc.h \
	navigation.cpp \
	serial.cpp serial.h \
	timing.cpp timing.h \
	uplink.h uplink.cpp \
	util.cpp util.h

decoder_SOURCES = decoder.c
decoder_LDADD =

avionics_LDFLAGS = -nodefaultlibs
avionics_LDADD = -lm -lgcc -lc


EXTRA_DIST = Makefile.orig

bin_PROGRAMS = avionics decoder

avionics_SOURCES = \
	ahrs_main.c \
	avionics.c \
	control.c \
	globaldefs.h \
	imugps.c \
	matrix.c matrix.h \
	misc.c \
	navfunc.c navfunc.h \
	navmain.c \
	serial.c serial.h \
	uplink.c
avionics_LDADD = -lrt -lm

decoder_SOURCES = decoder.c
decoder_LDADD =

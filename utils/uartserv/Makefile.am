noinst_PROGRAMS = uartserv

uartserv_SOURCES = \
	uartserv.cpp \
	netbuffer.cpp netbuffer.h \
        netChannel.cpp netChannel.h \
	serial.cpp serial.h

uartserv_LDADD = \
        ../../src/comms/libcomms.a \
        ../../src/control/libcontrol.a \
        ../../src/util/libutil.a

AM_CPPFLAGS = -I$(VPATH)/../../src


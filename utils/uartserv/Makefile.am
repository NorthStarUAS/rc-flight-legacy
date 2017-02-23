noinst_PROGRAMS = uartserv

uartserv_SOURCES = \
	uartserv.cxx \
	netbuffer.cxx netbuffer.hxx \
        netChannel.cxx netChannel.h \
	serial.cxx serial.hxx

uartserv_LDADD = \
        ../../src/comms/libcomms.a \
        ../../src/control/libcontrol.a \
        ../../src/util/libutil.a

AM_CPPFLAGS = -I$(VPATH)/../../src


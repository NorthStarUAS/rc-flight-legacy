noinst_PROGRAMS = uartserv

uartserv_SOURCES = \
	uartserv.cxx \
	netbuffer.cxx netbuffer.hxx

uartserv_LDADD = \
        ../../src/comms/libcomms.a \
        ../../src/control/libcontrol.a \
	../../src/props/libsgprops.a \
        ../../src/util/libutil.a

AM_CPPFLAGS = -I$(VPATH)/../../src


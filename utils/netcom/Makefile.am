bin_PROGRAMS = uartserv

uartserv_SOURCES = \
	uartserv.cxx \
	netbuffer.cxx netbuffer.hxx

uartserv_LDADD = \
        $(top_builddir)/src/comms/libcomms.a \
        $(top_builddir)/src/control/libcontrol.a \
	$(top_builddir)/src/props/libsgprops.a \
        $(top_builddir)/src/util/libutil.a

INCLUDES = -I$(top_srcdir)/src


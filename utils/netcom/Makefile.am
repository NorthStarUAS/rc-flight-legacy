bin_PROGRAMS = netcom

netcom_SOURCES = \
	netcom.cxx \
	netbuffer.cxx netbuffer.hxx

netcom_LDADD = \
        $(top_builddir)/src/comms/libcomms.a \
        $(top_builddir)/src/control/libcontrol.a \
	$(top_builddir)/src/props/libsgprops.a \
        $(top_builddir)/src/util/libutil.a

INCLUDES = -I$(top_srcdir)/src


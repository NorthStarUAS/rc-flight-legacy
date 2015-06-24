bin_PROGRAMS = uglink

uglink_SOURCES = \
	base64.cxx base64.hxx \
	command.cxx command.hxx \
	current.cxx current.hxx \
	geo.cxx geo.hxx \
	iochannel.cxx iochannel.hxx \
	messages.cxx messages.hxx \
	opengc.hxx \
	push_udp.cxx push_udp.hxx \
	serial.cxx serial.hxx \
	sg_file.cxx sg_file.hxx \
	telnet.cxx telnet.hxx \
        uglink.cxx \
	websocket.cxx websocket.hxx \
	wp_mgr.cxx wp_mgr.hxx

#-lsgio -lsgserial -lsgtiming -lsgmath -lsgprops -lsgstructure \
#	-lsgmisc -lsgdebug -lsgxml \
#        -lplibnet -lplibul

uglink_LDADD = \
	$(top_builddir)/src/math/libmath.a \
	$(top_builddir)/src/comms/libcomms.a \
	$(top_builddir)/src/props/libsgprops.a \
	$(top_builddir)/src/util/libutil.a \
	$(top_builddir)/src/xml/libsgxml.a \
	-lcrypto -lz


AM_CPPFLAGS = -I$(VPATH)/../src

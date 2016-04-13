noinst_LIBRARIES = libcomms.a

libcomms_a_SOURCES = \
	checksum.cxx checksum.hxx \
	display.cxx display.hxx \
	events.cxx events.hxx \
	logging.cxx logging.hxx \
        netBuffer.cxx netBuffer.h \
        netChannel.cxx netChannel.h \
        netChat.cxx netChat.h \
        netSocket.cxx netSocket.h ul.h \
	packer.cxx packer.hxx \
	packetizer.cxx packetizer.hxx \
	remote_link.cxx remote_link.hxx \
	serial.cxx serial.hxx \
	telnet.cxx telnet.hxx

AM_CPPFLAGS = -I$(VPATH)/.. -I$(VPATH)/../.. @PYTHON_INCLUDES@

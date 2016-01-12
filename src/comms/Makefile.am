noinst_LIBRARIES = libcomms.a

libcomms_a_SOURCES = \
	checksum.cpp checksum.h \
	display.cpp display.h \
	logging.cpp logging.h \
        netBuffer.cxx netBuffer.h \
        netChannel.cxx netChannel.h \
        netChat.cxx netChat.h \
        netSocket.cxx netSocket.h ul.h \
	packetizer.cxx packetizer.hxx \
	remote_link.cpp remote_link.h \
	serial.cxx serial.hxx \
	telnet.cxx telnet.hxx

AM_CPPFLAGS = -I$(VPATH)/.. -I$(VPATH)/../.. @PYTHON_INCLUDES@

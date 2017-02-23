noinst_LIBRARIES = libcomms.a

libcomms_a_SOURCES = \
	display.cxx display.hxx \
	events.cxx events.hxx \
	logging.cxx logging.hxx \
        netBuffer.cxx netBuffer.h \
        netChannel.cxx netChannel.h \
        netChat.cxx netChat.h \
        netSocket.cxx netSocket.h ul.h \
	packer.cxx packer.hxx \
	remote_link.cxx remote_link.hxx

AM_CPPFLAGS = -I$(VPATH)/.. -I$(VPATH)/../.. @PYTHON_INCLUDES@

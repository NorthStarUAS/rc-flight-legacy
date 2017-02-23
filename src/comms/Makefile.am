noinst_LIBRARIES = libcomms.a

libcomms_a_SOURCES = \
	display.cxx display.hxx \
	events.cxx events.hxx \
	logging.cxx logging.hxx \
        netSocket.cxx netSocket.h ul.h \
	packer.cxx packer.hxx \
	remote_link.cxx remote_link.hxx

AM_CPPFLAGS = -I$(VPATH)/.. -I$(VPATH)/../.. @PYTHON_INCLUDES@

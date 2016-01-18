SUBDIRS = \
	curt \
	umngnss_euler \
	umngnss_quat

noinst_LIBRARIES = libfilters.a

libfilters_a_SOURCES = \
	filter_mgr.cxx filter_mgr.hxx

AM_CPPFLAGS = -I$(VPATH)/.. -I$(VPATH)/../.. @PYTHON_INCLUDES@

SUBDIRS = nav_common nav_ekf15 nav_ekf15_mag

noinst_LIBRARIES = libfilters.a

libfilters_a_SOURCES = \
	filter_mgr.cxx filter_mgr.hxx \
	ground.cxx ground.hxx \
	wind.cxx wind.hxx

AM_CPPFLAGS = $(PYTHON_INCLUDES) -I$(VPATH)/.. -I$(VPATH)/../..

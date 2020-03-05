SUBDIRS = nav_common nav_ekf15 nav_ekf15_mag

noinst_LIBRARIES = libfilters.a

libfilters_a_SOURCES = \
	filter_mgr.cpp filter_mgr.h \
	ground.cpp ground.h \
	wind.cpp wind.h

AM_CPPFLAGS = $(PYTHON_INCLUDES) -I$(VPATH)/.. -I$(VPATH)/../..

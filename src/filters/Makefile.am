SUBDIRS = nav_common nav_eigen nav_eigen_mag

noinst_LIBRARIES = libfilters.a

libfilters_a_SOURCES = \
	filter_mgr.cxx filter_mgr.hxx \
	ground.cxx ground.hxx \
	wind.cxx wind.hxx

AM_CPPFLAGS = -I$(VPATH)/.. -I$(VPATH)/../..

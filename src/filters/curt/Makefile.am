noinst_LIBRARIES = libadns_curt.a

libadns_curt_a_SOURCES = \
        adns_curt.cxx adns_curt.hxx \
	glocal.cxx glocal.hxx

noinst_PROGRAMS = curtquat

curtquat_SOURCES = \
	curtquat.cpp

AM_CPPFLAGS = -I$(VPATH)/.. -I$(VPATH)/../.. @PYTHON_INCLUDES@

noinst_LIBRARIES = libadns_curt.a

libadns_curt_a_SOURCES = \
        adns_curt.cpp adns_curt.h \
	glocal.cpp glocal.h

noinst_PROGRAMS = curtquat

curtquat_SOURCES = \
	curtquat.cpp

AM_CPPFLAGS = -I$(VPATH)/.. -I$(VPATH)/../.. @PYTHON_INCLUDES@

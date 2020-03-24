SUBDIRS = Aura4

noinst_LIBRARIES = libdrivers.a

libdrivers_a_SOURCES = \
	driver.h \
	driver_mgr.cpp driver_mgr.h \
	airdata.cpp airdata.h \
	fgfs.cpp fgfs.h \
	gps.cpp gps.h \
	gps_ublox6.cpp gps_ublox6.h \
	gps_ublox8.cpp gps_ublox8.h \
	pilot_mgr.cpp pilot_mgr.h

bin_PROGRAMS = ublox-config

ublox_config_SOURCES = gps_ublox_config.cpp

ublox_config_LDADD = ../util/libutil.a

AM_CPPFLAGS = $(PYTHON_INCLUDES) -I$(VPATH)/.. -I$(VPATH)/../..

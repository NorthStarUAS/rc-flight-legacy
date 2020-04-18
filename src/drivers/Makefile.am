SUBDIRS = Aura4

noinst_LIBRARIES = libdrivers.a

libdrivers_a_SOURCES = \
	driver.h \
	driver_mgr.cpp driver_mgr.h \
	airdata.cpp airdata.h \
	fgfs.cpp fgfs.h \
	gps.cpp gps.h \
	lightware.cpp lightware.h \
	pilot.cpp pilot.h \
	gps_ublox6.cpp gps_ublox6.h \
	gps_ublox8.cpp gps_ublox8.h

AM_CPPFLAGS = $(PYTHON_INCLUDES) -I$(VPATH)/.. -I$(VPATH)/../..

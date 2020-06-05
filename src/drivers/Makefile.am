SUBDIRS = Aura4

noinst_LIBRARIES = libdrivers.a

libdrivers_a_SOURCES = \
	driver.h \
	driver_mgr.cpp driver_mgr.h \
	airdata.cpp airdata.h \
	fgfs.cpp fgfs.h \
	gps.cpp gps.h \
	gps_gpsd.cpp gps_gpsd.h \
	lightware.cpp lightware.h \
	maestro.cpp maestro.h \
	pilot.cpp pilot.h \
	ublox6.cpp ublox6.h \
	ublox8.cpp ublox8.h \
	ublox9.cpp ublox9.h

AM_CPPFLAGS = $(PYTHON_INCLUDES) -I$(VPATH)/.. -I$(VPATH)/../..

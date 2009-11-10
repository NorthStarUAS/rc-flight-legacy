if ENABLE_MNAV_SENSOR
    MNAV_SENSOR = mnav.cpp mnav.h
else
    MNAV_SENSOR =
endif

noinst_LIBRARIES = libsensors.a

libsensors_a_SOURCES = \
	airdata_mgr.cpp airdata_mgr.h \
	gps_mgr.cpp gps_mgr.h \
	gpsd.cpp gpsd.h \
	$(MNAV_SENSOR) \
	ugfile.cpp ugfile.h

INCLUDES = -I$(top_srcdir)/src

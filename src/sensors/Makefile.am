if ENABLE_MNAV_SENSOR
    MNAV_SENSOR = mnav.cpp mnav.h
else
    MNAV_SENSOR =
endif

noinst_LIBRARIES = libsensors.a

libsensors_a_SOURCES = \
	airdata_mgr.cpp airdata_mgr.h \
	imu_fgfs.cxx imu_fgfs.hxx \
	sf_6DOFv4.cxx sf_6DOFv4.h \
	gps_mgr.cpp gps_mgr.h \
	gpsd.cpp gpsd.h \
	gps_fgfs.cxx gps_fgfs.hxx \
	gps_ublox5.cxx gps_ublox5.h \
	$(MNAV_SENSOR) \
	ugfile.cpp ugfile.h

INCLUDES = -I$(top_srcdir)/src

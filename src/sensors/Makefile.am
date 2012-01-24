if ENABLE_MNAV_SENSOR
    MNAV_SENSOR = mnav.cxx mnav.hxx
else
    MNAV_SENSOR =
endif

noinst_LIBRARIES = libsensors.a

libsensors_a_SOURCES = \
	airdata_mgr.cxx airdata_mgr.hxx \
	imu_fgfs.cxx imu_fgfs.hxx \
	imu_sf6DOFv4.cxx imu_sf6DOFv4.hxx \
	imu_vn100.cxx imu_vn100.hxx \
	gps_mgr.cxx gps_mgr.hxx \
	gps_gpsd.cxx gps_gpsd.hxx \
	gps_fgfs.cxx gps_fgfs.hxx \
	gps_mediatek.cxx gps_mediatek.hxx \
	gps_ublox5.cxx gps_ublox5.hxx \
	$(MNAV_SENSOR) \
	pilot_mgr.cxx pilot_mgr.hxx \
	pilot_fgfs.cxx pilot_fgfs.hxx \
	ardupilot.cxx ardupilot.hxx \
	ugfile.cxx ugfile.hxx

INCLUDES = -I$(top_srcdir)/src -I$(top_builddir)/src

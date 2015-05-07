noinst_LIBRARIES = libsensors.a

libsensors_a_SOURCES = \
	airdata_mgr.cxx airdata_mgr.hxx \
	calibrate.hxx calibrate.cxx \
	imu_fgfs.cxx imu_fgfs.hxx \
	imu_sf6DOFv4.cxx imu_sf6DOFv4.hxx \
	imu_vn100_uart.cxx imu_vn100_uart.hxx \
	imu_vn100_spi.cxx imu_vn100_spi.hxx \
	gps_mgr.cxx gps_mgr.hxx \
	gps_gpsd.cxx gps_gpsd.hxx \
	gps_fgfs.cxx gps_fgfs.hxx \
	gps_mediatek.cxx gps_mediatek.hxx \
	gps_ublox.cxx gps_ublox.hxx \
	pilot_mgr.cxx pilot_mgr.hxx \
	pilot_fgfs.cxx pilot_fgfs.hxx \
	ardupilot.cxx ardupilot.hxx \
	APM2.cxx APM2.hxx \
	ugfile.cxx ugfile.hxx

bin_PROGRAMS = ublox-config

ublox_config_SOURCES = gps_ublox_config.cxx

ublox_config_LDADD = \
	$(top_builddir)/src/math/libmath.a \
	$(top_builddir)/src/util/libutil.a

INCLUDES = -I$(top_srcdir)/src -I$(top_builddir)/src

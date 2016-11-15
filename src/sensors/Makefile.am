noinst_LIBRARIES = libsensors.a

libsensors_a_SOURCES = \
	airdata_mgr.cxx airdata_mgr.hxx \
	airdata_bolder.cxx airdata_bolder.hxx \
	cal_temp.hxx cal_temp.cxx \
        imu_mgr.cxx imu_mgr.hxx \
	imu_vn100_uart.cxx imu_vn100_uart.hxx \
	imu_vn100_spi.cxx imu_vn100_spi.hxx \
	gps_mgr.cxx gps_mgr.hxx \
	gps_gpsd.cxx gps_gpsd.hxx \
	gps_mediatek.cxx gps_mediatek.hxx \
	gps_ublox6.cxx gps_ublox6.hxx \
	gps_ublox8.cxx gps_ublox8.hxx \
	pilot_mgr.cxx pilot_mgr.hxx \
	APM2.cxx APM2.hxx \
	FGFS.cxx FGFS.hxx \
	Goldy2.cxx Goldy2.hxx \
	pika.hxx pika.cxx \
	raven1.hxx raven1.cxx \
	raven2.hxx raven2.cxx \
	ugfile.cxx ugfile.hxx \
	util_goldy2.cxx util_goldy2.hxx

bin_PROGRAMS = ublox-config

ublox_config_SOURCES = gps_ublox_config.cxx

ublox_config_LDADD = \
	../math/libmath.a \
	../util/libutil.a

AM_CPPFLAGS = -I$(VPATH)/.. -I$(VPATH)/../.. @PYTHON_INCLUDES@

SUBDIRS = Aura3

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
	gps_ublox6.cxx gps_ublox6.hxx \
	gps_ublox8.cxx gps_ublox8.hxx \
	pilot_mgr.cxx pilot_mgr.hxx \
	APM2.cxx APM2.hxx \
	FGFS.cxx FGFS.hxx \
	ugfile.cxx ugfile.hxx

bin_PROGRAMS = ublox-config

ublox_config_SOURCES = gps_ublox_config.cxx

ublox_config_LDADD = ../util/libutil.a

AM_CPPFLAGS = $(PYTHON_INCLUDES) -I$(VPATH)/.. -I$(VPATH)/../..

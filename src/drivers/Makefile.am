SUBDIRS = Aura3 Aura4

noinst_LIBRARIES = libsensors.a

libsensors_a_SOURCES = \
	driver.h \
	driver_mgr.cpp driver_mgr.h \
	airdata_mgr.cpp airdata_mgr.h \
	airdata_bolder.cpp airdata_bolder.h \
	cal_temp.h cal_temp.cpp \
        imu_mgr.cpp imu_mgr.h \
	imu_vn100_uart.cpp imu_vn100_uart.h \
	imu_vn100_spi.cpp imu_vn100_spi.h \
	gps_mgr.cpp gps_mgr.h \
	gps_gpsd.cpp gps_gpsd.h \
	gps_ublox6.cpp gps_ublox6.h \
	gps_ublox8.cpp gps_ublox8.h \
	pilot_mgr.cpp pilot_mgr.h \
	APM2.cpp APM2.h \
	FGFS.cpp FGFS.h \
	ugfile.cpp ugfile.h

bin_PROGRAMS = ublox-config

ublox_config_SOURCES = gps_ublox_config.cpp

ublox_config_LDADD = ../util/libutil.a

AM_CPPFLAGS = $(PYTHON_INCLUDES) -I$(VPATH)/.. -I$(VPATH)/../..

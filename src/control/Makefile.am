noinst_LIBRARIES = libcontrol.a

libcontrol_a_SOURCES = \
	actuators.cpp actuators.h \
	ap.cpp ap.h \
	cas.cpp cas.h \
	component.h \
	control.cpp control.h \
	dig_filter.cpp dig_filter.h \
	dtss.cpp dtss.h \
	pid.cpp pid.h \
	pid_vel.cpp pid_vel.h \
	predictor.cpp predictor.h \
	summer.cpp summer.h \
	tecs.cpp tecs.h

AM_CPPFLAGS = $(PYTHON_INCLUDES) -I$(VPATH)/.. -I$(VPATH)/../..

noinst_LIBRARIES = libutil.a

libutil_a_SOURCES = \
	butter.cpp butter.h \
	cal_temp.h cal_temp.cpp \
	geodesy.cpp geodesy.h \
	linearfit.cpp linearfit.h \
	lowpass.cpp lowpass.h \
	myprof.cpp myprof.h \
	poly1d.h \
	props_helper.cpp proper_helper.h \
	sg_path.cpp sg_path.h \
	strutils.cpp strutils.h \
        timing.cpp timing.h \
        netSocket.cpp netSocket.h ul.h

AM_CPPFLAGS = $(PYTHON_INCLUDES) -I$(VPATH)/.. -I$(VPATH)/../..

noinst_PROGRAMS = butter_test # geodesy_test

butter_test_SOURCES = butter_test.cpp
butter_test_LDADD = libutil.a

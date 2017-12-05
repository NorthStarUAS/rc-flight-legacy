noinst_LIBRARIES = libutil.a

libutil_a_SOURCES = \
	butter.cxx butter.hxx \
	coremag.c coremag.h \
	exception.cxx exception.hxx \
	linearfit.cxx linearfit.hxx \
	lowpass.cxx lowpass.hxx \
	myprof.cxx myprof.h \
	poly1d.hxx \
	sg_inlines.h \
	sg_path.cxx sg_path.hxx \
	SGReferenced.hxx SGSharedPtr.hxx \
	strutils.hxx strutils.cxx \
        timing.cpp timing.h \
        netSocket.cxx netSocket.h ul.h

AM_CPPFLAGS = -I$(VPATH)/.. -I$(VPATH)/../..

noinst_PROGRAMS = butter_test

butter_test_SOURCES = butter_test.cxx
butter_test_LDADD = libutil.a

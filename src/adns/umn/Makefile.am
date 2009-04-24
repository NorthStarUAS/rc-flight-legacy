EXTRA_DIST = testscript.sh

noinst_LIBRARIES = libadns_umn.a
libadns_umn_a_SOURCES = \
	adns.c adns.h \
	earth.h earth.c \
	eulerdcm.h eulerdcm.c \
	kalmanfilter.h kalmanfilter.c \
	ivl_matrix.h ivl_matrix.c \
	ivl_mem.h ivl_mem.c \
	matrix.h matrix.c \
	strapdown.h strapdown.c \
	worldcoord.h worldcoord.c

noinst_PROGRAMS = test_closedloop nexttest

test_closedloop_LDADD = $(top_builddir)/src/adns_umn/libadns_umn.a -lm
#newtest_SOURCES = newtest.c
#newtest_LDADD = $(top_builddir)/src/adns_umn/libadns_umn.a -lm
nexttest_SOURCES = nexttest.c
nexttest_LDADD = $(top_builddir)/src/adns_umn/libadns_umn.a -lm

INCLUDES = -I$(top_srcdir)/src

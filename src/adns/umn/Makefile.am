EXTRA_DIST = testscript.sh

noinst_LIBRARIES = libadns_umn.a
libadns_umn_a_SOURCES = \
	earth.h earth.c \
	eulerdcm.h eulerdcm.c \
	kalmanfilter.h kalmanfilter.c \
	ivl_matrix.h ivl_matrix.c \
	ivl_mem.h ivl_mem.c \
	matrix.h matrix.c \
	strapdown.h strapdown.c \
	worldcoord.h worldcoord.c

noinst_PROGRAMS = test_closedloop
test_closedloop_LDADD = $(top_builddir)/src/adns_umn/libadns_umn.a -lm

INCLUDES = -I$(top_srcdir)/src

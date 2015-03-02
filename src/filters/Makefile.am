if ENABLE_MNAV_FILTER
MNAV_DIR = mnav
else
MNAV_DIR =
endif

SUBDIRS = \
	curt $(MNAV_DIR) umngnss_euler

noinst_LIBRARIES = libfilters.a

libfilters_a_SOURCES = \
	filter_mgr.cpp filter_mgr.h

INCLUDES = -I$(top_srcdir)/src -I$(top_builddir)/src

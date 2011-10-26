if ENABLE_MNAV_FILTER
MNAV_DIR = mnav
else
MNAV_DIR =
endif

if ENABLE_GNSS_FILTER
GNSS_DIR = gnss
else
GNSS_DIR =
endif

SUBDIRS = \
	curt $(MNAV_DIR) $(GNSS_DIR)

noinst_LIBRARIES = libfilters.a

libfilters_a_SOURCES = \
	filter_mgr.cpp filter_mgr.h

INCLUDES = -I$(top_srcdir)/src

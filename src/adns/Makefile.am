if ENABLE_MNAV_FILTER
MNAV_DIR = mnav
else
MNAV_DIR =
endif

SUBDIRS = \
	curt $(MNAV_DIR)

noinst_LIBRARIES = libadns.a

libadns_a_SOURCES = \
	adns_mgr.cpp adns_mgr.h \
	umn_interface.cpp umn_interface.h

INCLUDES = -I$(top_srcdir)/src

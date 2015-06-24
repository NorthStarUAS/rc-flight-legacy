if ENABLE_MNAV_FILTER
MNAV_DIR = mnav
else
MNAV_DIR =
endif

SUBDIRS = \
	curt \
	$(MNAV_DIR) \
	umngnss_euler \
	umngnss_quat

noinst_LIBRARIES = libfilters.a

libfilters_a_SOURCES = \
	filter_mgr.cpp filter_mgr.h

AM_CPPFLAGS = -I$(VPATH)/.. -I$(VPATH)/../..

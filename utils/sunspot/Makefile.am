#noinst_PROGRAMS = dynamichome shadowpoint
noinst_PROGRAMS = dynamichome

dynamichome_SOURCES = \
	main.cpp

dynamichome_LDADD = \
	$(top_builddir)/src/comms/libcomms.a \
	$(top_builddir)/src/math/libmath.a \
	$(top_builddir)/src/util/libutil.a

#shadowpoint_SOURCES = \
#	shadowpoint.cpp

#shadowpoint_LDADD = \
#	-lsgmath -lsgio -lsgephem -lsgtiming -lsgmisc -lsgstructure -lplibnet -lsgdebug

#	-lsgtiming -lsgmisc -lsgdebug -lplibnet -lplibul \
#	$(joystick_LIBS) $(network_LIBS) $(base_LIBS) -lz

INCLUDES = -I$(top_srcdir)/src

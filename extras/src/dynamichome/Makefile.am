noinst_PROGRAMS = dynamichome dynamichome_old
#bin_PROGRAMS = dynamichome

dynamichome_old_SOURCES = \
	main.cpp

dynamichome_old_LDADD = \
	$(top_builddir)/src/comms/libcomms.a \
	$(top_builddir)/src/math/libmath.a \
	$(top_builddir)/src/util/libutil.a

dynamichome_SOURCES = \
	dynamichome.cpp

dynamichome_LDADD = \
	$(top_builddir)/src/comms/libcomms.a \
	$(top_builddir)/src/math/libmath.a \
	$(top_builddir)/src/util/libutil.a -lgps

#shadowpoint_SOURCES = \
#	shadowpoint.cpp

#shadowpoint_LDADD = \
#	-lsgmath -lsgio -lsgephem -lsgtiming -lsgmisc -lsgstructure -lplibnet -lsgdebug

#	-lsgtiming -lsgmisc -lsgdebug -lplibnet -lplibul \
#	$(joystick_LIBS) $(network_LIBS) $(base_LIBS) -lz

AM_CPPFLAGS = -I$(top_srcdir)/src

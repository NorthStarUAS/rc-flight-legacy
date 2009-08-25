dnl Process this file with autogen.sh to produce a working configure
dnl script.

dnl Initialize the autoconf system
AC_INIT

dnl Require at least automake 2.52
AC_PREREQ(2.52)

dnl Sanity check of the distribution
AC_CONFIG_SRCDIR([src/main/ugear.cpp])

dnl Initialize the automake system (provide project name and version)
AM_INIT_AUTOMAKE(MicroGear, 1.8)

dnl Checks for programs.
AC_PROG_MAKE_SET
AC_PROG_CC
AC_PROG_CPP
AC_PROG_CXX
AC_PROG_RANLIB
AC_PROG_INSTALL
AC_PROG_LN_S

dnl set the $host variable based on local machine/os
AC_CANONICAL_HOST

dnl use the acinclude macro to find extra include directories.
wi_EXTRA_DIRS(no, ${EXTRA_DIRS})

dnl check for default libraries
AC_SEARCH_LIBS(clock_gettime, [rt])
AC_SEARCH_LIBS(cos, [m])
AC_SEARCH_LIBS(gzopen, [z])

AM_CONFIG_HEADER(src/include/ugear_config.h)

AC_CONFIG_FILES([ \
	Makefile \
        src/Makefile \
        src/actuators/Makefile \
        src/adns/Makefile \
        src/adns/mnav/Makefile \
        src/adns/umn/Makefile \
        src/comms/Makefile \
        src/control/Makefile \
        src/health/Makefile \
        src/include/Makefile \
        src/main/Makefile \
        src/props/Makefile \
        src/sensors/Makefile \
        src/util/Makefile \
        src/xml/Makefile \
        utils/Makefile \
        utils/benchmarks/Makefile \
        utils/routegen/Makefile \
])
AC_OUTPUT

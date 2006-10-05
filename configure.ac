dnl Process this file with autogen.sh to produce a working configure
dnl script.

dnl Initialize the autoconf system
AC_INIT

dnl Require at least automake 2.52
AC_PREREQ(2.52)

dnl Sanity check of the distribution
AC_CONFIG_SRCDIR([src/avionics.c])

dnl Initialize the automake system (provide project name and version)
AM_INIT_AUTOMAKE(MicroNav, 1.3)

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

dnl Thread related checks
AC_CHECK_HEADER(pthread.h)
if test "x$ac_cv_header_pthread_h" = "xyes"; then
  CXXFLAGS="$CXXFLAGS -pthread"
  CFLAGS="$CFLAGS -pthread"
else
  echo "Unable to find a working pthreads implimentation."
  echo "configure aborted."
  exit
fi

dnl check for default libraries
AC_SEARCH_LIBS(clock_nanosleep, [rt])
AC_SEARCH_LIBS(cos, [m])

AM_CONFIG_HEADER(src/unav_config.h)

AC_CONFIG_FILES([ \
	Makefile \
        src/Makefile \
])
AC_OUTPUT

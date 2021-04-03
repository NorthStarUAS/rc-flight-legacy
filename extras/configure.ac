dnl Process this file with autogen.sh to produce a working configure
dnl script.

dnl Require at least automake 2.52
AC_PREREQ(2.52)

dnl Initialize the autoconf system (provide project name and version)
AC_INIT([rcUAS-extras], [2.3])

dnl Sanity check of the distribution
AC_CONFIG_SRCDIR([src/gps_config/gps_ublox_config.cpp])

dnl Initialize the automake system
AM_INIT_AUTOMAKE

dnl Checks for programs.
AC_PROG_MAKE_SET
AC_PROG_CC
AC_PROG_CPP
AC_PROG_CXX
AC_PROG_RANLIB
AC_PROG_INSTALL
AC_PROG_LN_S

echo "AR is $AR"
if test -z "${AR+set}"; then
  AR=ar
fi
echo "AR is $AR"
AC_SUBST(AR)

dnl set the $host variable based on local machine/os
AC_CANONICAL_HOST

dnl use the acinclude macro to find extra include directories.
wi_EXTRA_DIRS(no, ${EXTRA_DIRS})

dnl check for default libraries
AC_SEARCH_LIBS(clock_gettime, [rt])
AC_SEARCH_LIBS(cos, [m])
AC_SEARCH_LIBS(gzopen, [z])

dnl check for Eigen C++ matrix/vector/quaternion headers
AC_LANG_SAVE
AC_LANG_CPLUSPLUS
AC_CHECK_HEADERS([eigen3/Eigen/Core])
if test "x$ac_cv_header_eigen3_Eigen_Core" != xyes; then
   echo "configure error: Eigen matrix headers are required"
   exit
fi
AC_LANG_RESTORE

AM_CONFIG_HEADER(src/extras_config.h)

AC_CONFIG_FILES([ \
	Makefile \
	src/Makefile \
        src/gps_config/Makefile \
        src/autohome/Makefile \
        src/benchmarks/Makefile \
        src/dynamichome/Makefile \
        src/uartlogger/Makefile \
        src/uartserv/Makefile \
])
AC_OUTPUT


dnl Process this file with autogen.sh to produce a working configure
dnl script.

dnl Require at least automake 2.52
AC_PREREQ(2.52)

dnl Initialize the autoconf system (provide project name and version)
AC_INIT([aura-core], [2.1])

dnl Sanity check of the distribution
AC_CONFIG_SRCDIR([src/init/globals.hxx])

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

dnl find python primary
AM_PATH_PYTHON
AC_PATH_PROGS([PYTHON_CONFIG],
              [python$PYTHON_VERSION-config python-config],
              [no],
              [`dirname $PYTHON`])
AS_IF([test "$PYTHON_CONFIG" = no], [AC_MSG_ERROR([cannot find python-config for $PYTHON.])])
PYTHON_INCLUDES=`$PYTHON_CONFIG --includes`
AC_MSG_RESULT([$PYTHON_INCLUDES])
AC_SUBST(PYTHON_INCLUDES)
PYTHON_LIBS=`$PYTHON_CONFIG --libs`
AC_MSG_RESULT([$PYTHON_LIBS])
AC_SUBST(PYTHON_LIBS)

dnl check for Eigen C++ matrix/vector/quaternion library
AC_LANG_SAVE
AC_LANG_CPLUSPLUS
AC_CHECK_HEADERS([eigen3/Eigen/Core])
AC_LANG_RESTORE
if test "x$ac_cv_header_eigen3_Eigen_Core" != xyes; then
   echo "configure error: eigen3 matrix libraries are required"
   exit
fi

AM_CONFIG_HEADER(src/include/aura_config.h)

dnl automatic detection of pointing library

dnl code configuration selections

AC_ARG_ENABLE(batch-mode, [  --enable-batch-mode    Disable real-time clock/timing for running in batch mode], [enable_batch_mode="$enableval"], [enable_batch_mode="no"] )
if test "x$enable_batch_mode" != "xno"; then
    AC_DEFINE([BATCH_MODE], 1, [Define to disable real time timing and run flat out])
fi

dnl code module selections

AC_CONFIG_FILES([ \
	Makefile \
        src/Makefile \
        src/actuators/Makefile \
        src/filters/Makefile \
        src/filters/nav_eigen/Makefile \
        src/filters/umngnss_quat/Makefile \
        src/comms/Makefile \
        src/control/Makefile \
        src/health/Makefile \
        src/include/Makefile \
        src/init/Makefile \
	src/main/Makefile \
	src/math/Makefile \
	src/mission/Makefile \
        src/payload/Makefile \
        src/python/Makefile \
        src/sensors/Makefile \
        src/util/Makefile \
        uglink/Makefile \
        utils/Makefile \
        utils/admin/Makefile \
        utils/autohome/Makefile \
        utils/benchmarks/Makefile \
        utils/dynamichome/Makefile \
        utils/geo/Makefile \
        utils/routegen/Makefile \
        utils/uartserv/Makefile \
])
AC_OUTPUT


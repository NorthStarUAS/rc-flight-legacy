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

AM_CONFIG_HEADER(src/include/ugear_config.h)

dnl automatic detection of pointing library

dnl code configuration selections

AC_ARG_ENABLE(batch-mode, [  --enable-batch-mode    Disable real-time clock/timing for running in batch mode], [enable_batch_mode="$enableval"], [enable_batch_mode="no"] )
if test "x$enable_batch_mode" != "xno"; then
    AC_DEFINE([BATCH_MODE], 1, [Define to disable real time timing and run flat out])
fi


dnl code module selections

LGPL_CLEAN="yes"

AC_CONFIG_FILES([ \
	Makefile \
        src/Makefile \
        src/actuators/Makefile \
        src/filters/Makefile \
        src/filters/curt/Makefile \
        src/filters/umngnss_euler/Makefile \
        src/filters/umngnss_quat/Makefile \
        src/comms/Makefile \
        src/control/Makefile \
        src/health/Makefile \
        src/include/Makefile \
        src/init/Makefile \
	src/math/Makefile \
        src/payload/Makefile \
        src/props/Makefile \
        src/sensors/Makefile \
        src/util/Makefile \
        src/xml/Makefile \
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

echo
echo LGPL_CLEAN = $LGPL_CLEAN

if test "x$LGPL_CLEAN" = "xno"; then
    cat <<LGPLEOF

NOTICE: you have included certain modules in this configuration that
are not LGPL compatible.

You can use the --help option to configure for a complete list of
available options.

LGPLEOF
else
    cat <<LGPLEOF

You have configured an LGPL clean build and may link these libraries
into a larger proprietary application if desired.

LGPLEOF
fi

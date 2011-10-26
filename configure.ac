dnl Process this file with autogen.sh to produce a working configure
dnl script.

dnl Initialize the autoconf system
AC_INIT

dnl Require at least automake 2.52
AC_PREREQ(2.52)

dnl Sanity check of the distribution
AC_CONFIG_SRCDIR([src/main/ugear_gumstix.cpp])

dnl Initialize the automake system (provide project name and version)
AM_INIT_AUTOMAKE(MicroGear, 1.9)

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

AC_SEARCH_LIBS(umn_adns_init, [umngnss])

AM_CONFIG_HEADER(src/include/ugear_config.h)


dnl automatic detection of pointing library

AC_LANG_PUSH(C++)
AC_CHECK_HEADER(atipointing.hxx)
if test "x$ac_cv_header_atipointing_hxx" = "xyes"; then
    AC_DEFINE([ATI_POINTING], 1, [Define to include ATI pointing module.])
fi
AM_CONDITIONAL(ENABLE_ATI_POINTING, test "x$ac_cv_header_atipointing_hxx" = "xyes")
AC_LANG_POP


dnl code configuration selections

AC_ARG_ENABLE(batch-mode, [  --enable-batch-mode    Disable real-time clock/timing for running in batch mode], [enable_batch_mode="$enableval"], [enable_batch_mode="no"] )
if test "x$enable_batch_mode" != "xno"; then
    AC_DEFINE([BATCH_MODE], 1, [Define to disable real time timing and run flat out])
fi


dnl code module selections

GPL_CLEAN="yes"

AC_ARG_ENABLE(gpl-modules, [  --enable-gpl-modules    Include all GPL module code], [enable_gpl_modules="$enableval"], [enable_gpl_modules="no"] )
if test "x$enable_gpl_modules" != "xno"; then
    GPL_CLEAN="no"
    enable_mnav_sensor="yes"
    enable_mnav_filter="yes"
fi

AC_ARG_ENABLE(mnav-sensor, [  --enable-mnav-sensor    Include MNAV sensor driver], [enable_mnav_sensor="$enableval"], [enable_mnav_sensor="no"] )
if test "x$enable_mnav_sensor" != "xno"; then
    AC_DEFINE([ENABLE_MNAV_SENSOR], 1, [Define to include MNAV sensor driver])
    GPL_CLEAN="no"
fi
AM_CONDITIONAL(ENABLE_MNAV_SENSOR, test "x$enable_mnav_sensor" != "xno")

AC_ARG_ENABLE(mnav-filter, [  --enable-mnav-filter    Include MNAV filter], [enable_mnav_filter="$enableval"], [enable_mnav_filter="no"] )
if test "x$enable_mnav_filter" != "xno"; then
    AC_DEFINE([ENABLE_MNAV_FILTER], 1, [Define to include MNAV filter])
    GPL_CLEAN="no"
fi
AM_CONDITIONAL(ENABLE_MNAV_FILTER, test "x$enable_mnav_filter" != "xno")

AC_ARG_ENABLE(gnss-filter, [  --enable-gnss-filter    Include Advanced GNSS filter], [enable_gnss_filter="$enableval"], [enable_gnss_filter="no"] )
if test "x$enable_gnss_filter" != "xno"; then
    AC_DEFINE([ENABLE_GNSS_FILTER], 1, [Define to include Advnaced GNSS filter])
fi
AM_CONDITIONAL(ENABLE_GNSS_FILTER, test "x$enable_gnss_filter" != "xno")

AC_CONFIG_FILES([ \
	Makefile \
        src/Makefile \
        src/actuators/Makefile \
        src/filters/Makefile \
        src/filters/curt/Makefile \
        src/filters/gnss/Makefile \
        src/filters/mnav/Makefile \
        src/comms/Makefile \
        src/control/Makefile \
        src/health/Makefile \
        src/include/Makefile \
        src/main/Makefile \
        src/math/Makefile \
        src/props/Makefile \
        src/sensors/Makefile \
        src/util/Makefile \
        src/xml/Makefile \
        utils/Makefile \
        utils/benchmarks/Makefile \
        utils/routegen/Makefile \
        utils/uartserv/Makefile \
])
AC_OUTPUT

echo
echo GPL_CLEAN = $GPL_CLEAN

if test "x$GPL_CLEAN" = "xno"; then
    cat <<GPLEOF

NOTICE: you have included certain GPL licensed modules in this
configuration.  The GPL license is "viral" so you will need to ensure
that all of your own code that is included in this build is licensed
with a GPL compatible license.

If this causes a problem, consider running "./configure" with the
"--disable-gpl-modules" option to exclude GPL code.  This produces a
configuration with only LGPL, MPL, or public domain code and allows
you to link in your own proprietary drivers or modules.

You can use the --help option to configure for a complete list of
available options.

GPLEOF
else
    cat <<GPLEOF

You have configured a "GPL clean" build and may link in proprietary
code modules if desired.

GPLEOF
fi

cat <<GPLEOF
MNAV sensor driver (GPL): $enable_mnav_sensor
MNAV filter (GPL)       : $enable_mnav_filter
GPLEOF

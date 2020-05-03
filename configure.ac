dnl Process this file with autogen.sh to produce a working configure
dnl script.

dnl Require at least automake 2.52
AC_PREREQ(2.52)

dnl Initialize the autoconf system (provide project name and version)
AC_INIT([aura-core], [2.2])

dnl Sanity check of the distribution
AC_CONFIG_SRCDIR([src/init/globals.h])

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

# dnl find python primary
# AM_PATH_PYTHON([3])
# AC_PATH_PROGS([PYTHON_CONFIG],
#               [python$PYTHON_VERSION-config python-config],
#               [no],
#               [`dirname $PYTHON`])
# AS_IF([test "$PYTHON_CONFIG" = no], [AC_MSG_ERROR([cannot find python-config for $PYTHON.])])
# PYTHON_INCLUDES=`$PYTHON_CONFIG --includes`
# PYTHON_INCLUDES=""
# AC_MSG_RESULT([$PYTHON_INCLUDES])
# AC_SUBST(PYTHON_INCLUDES)
# PYTHON_LIBS=`$PYTHON_CONFIG --libs`
# PYTHON_LIBS="-lpython2.7"
# AC_MSG_RESULT([$PYTHON_LIBS])
# AC_SUBST(PYTHON_LIBS)

# To make the build more straight forward (hopefully) on cross compiled
# devel environments, hard code the libpython libraries
# you can use python3-config --includes --libs to find this info
PYTHON_INCLUDES="-I/usr/include/python3.8"
PYTHON_LIBS="-lpyprops -lpython3.8"
AC_SUBST(PYTHON_INCLUDES)
AC_SUBST(PYTHON_LIBS)

dnl check for Eigen C++ matrix/vector/quaternion headers
AC_LANG_SAVE
AC_LANG_CPLUSPLUS
AC_CHECK_HEADERS([eigen3/Eigen/Core])
if test "x$ac_cv_header_eigen3_Eigen_Core" != xyes; then
   echo "configure error: Eigen matrix headers are required"
   exit
fi
AC_LANG_RESTORE

AM_CONFIG_HEADER(src/include/aura_config.h)

AC_CONFIG_FILES([ \
	Makefile \
        src/Makefile \
        src/drivers/Makefile \
        src/drivers/Aura4/Makefile \
        src/comms/Makefile \
        src/control/Makefile \
        src/filters/Makefile \
        src/filters/nav_common/Makefile \
        src/filters/nav_ekf15/Makefile \
        src/filters/nav_ekf15_mag/Makefile \
        src/health/Makefile \
        src/include/Makefile \
        src/init/Makefile \
        src/util/Makefile \
        tools/gps_config/Makefile \
        utils/Makefile \
        utils/autohome/Makefile \
        utils/benchmarks/Makefile \
        utils/dynamichome/Makefile \
        utils/uartlogger/Makefile \
        utils/uartserv/Makefile \
])
AC_OUTPUT


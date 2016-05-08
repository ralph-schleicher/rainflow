dnl Process this file with autoconf to produce a configure script.

AC_PREREQ([2.50])

AC_INIT([rs-rainflow], [20160508.1803])
AM_INIT_AUTOMAKE([foreign no-define no-dependencies])

AC_CONFIG_SRCDIR([rs-rainflow.h])
AC_CONFIG_HEADER([config.h])
AC_CONFIG_FILES([Makefile
		 doc/Makefile
		 t/Makefile
		 matlab/Makefile])

dnl Optional features.
AM_MAINTAINER_MODE

dnl Checks for programs.
AC_PROG_CC_C99
AC_PROG_INSTALL
AC_LIBTOOL_WIN32_DLL
AC_PROG_LIBTOOL

AC_CHECK_PROGS([MKOCTFILE], [mkoctfile], [true])

dnl Checks for libraries.
case $host in
  *-cygwin* | *-mingw*)
    ;;
  *)
    AC_SEARCH_LIBS([pow], [m])
    ;;
esac

dnl Checks for header files.

dnl Checks for types.

dnl Checks for compiler characteristics.

AC_SUBST([AM_CPPFLAGS], ['-I$(top_srcdir)'])

AC_SUBST([PACKAGE_NAME])
AC_SUBST([PACKAGE_VERSION])
AC_SUBST([PACKAGE_STRING])
AC_SUBST([PACKAGE_BUGREPORT])
AC_SUBST([PACKAGE_TARNAME])

AC_OUTPUT

dnl local variables:
dnl time-stamp-time-zone: "UTC"
dnl time-stamp-format: "%:y%02m%02d.%02H%02M"
dnl time-stamp-start: "AC_INIT(\\[[^]]*\\], \\["
dnl time-stamp-end: "\\]"
dnl end:

#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.61)
AC_INIT([robot], [1.1], [benjamin-james@users.noreply.github.com])
AC_CONFIG_SRCDIR([src/pc_demo.c])
AM_INIT_AUTOMAKE
AC_CONFIG_HEADER([config.h])

# Checks for programs.
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_MAKE_SET
LT_INIT

# Checks for libraries.

# Checks for header files.

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_TIME

# Checks for library functions.
AC_FUNC_MALLOC
AC_HEADER_STDC
AC_CHECK_FUNCS([gettimeofday sqrt])

AC_CONFIG_FILES([Makefile src/Makefile])
AC_OUTPUT

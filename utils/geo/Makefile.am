ugear_builddir = @UGEAR_BUILDDIR@

EXTRA_DIST = 

noinst_PROGRAMS = demo1

demo1_SOURCES = demo1.cxx

demo1_LDADD = \
	$(top_builddir)/src/math/libmath.a \
	$(top_builddir)/src/util/libutil.a

INCLUDES = -I$(top_srcdir)/src -I$(ugear_builddir)/src

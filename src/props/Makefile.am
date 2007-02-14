lib_LIBRARIES = libsgprops.a

include_HEADERS = \
	props.hxx \
	props_io.hxx

libsgprops_a_SOURCES = \
	props.cxx \
	props_io.cxx

noinst_PROGRAMS = props_test

props_test_SOURCES = props_test.cxx
props_test_LDADD = \
	$(top_builddir)/src/props/libsgprops.a \
	$(top_builddir)/src/xml/libsgxml.a \
	$(top_builddir)/src/util/libutil.a

INCLUDES = -I$(top_srcdir)/src

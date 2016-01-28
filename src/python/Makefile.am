noinst_LIBRARIES = libpyprops.a

include_HEADERS = \
	pyprops.hxx \
	python_sys.hxx

libpyprops_a_SOURCES = \
	pyprops.cxx \
	python_sys.cxx

AM_CPPFLAGS = -I$(VPATH)/.. -I$(VPATH)/../.. @PYTHON_INCLUDES@

noinst_PROGRAMS = props_test

props_test_SOURCES = props_test.cxx
props_test_LDADD = libpyprops.a @PYTHON_LIBS@


noinst_LIBRARIES = libpyprops.a

include_HEADERS = \
	pymodule.hxx \
	pyprops.hxx \
	python_sys.hxx

libpyprops_a_SOURCES = \
	pymodule.cxx \
	pyprops.cxx \
	python_sys.cxx

AM_CPPFLAGS = $(PYTHON_INCLUDES) -I$(VPATH)/.. -I$(VPATH)/../..

noinst_PROGRAMS = props_test

props_test_SOURCES = props_test.cxx
props_test_LDADD = libpyprops.a $(PYTHON_LIBS)


# For StarGate compile uncomment the following lines:
# ugear_LDFLAGS = -nodefaultlibs
# ugear_MORELIBS = -lm -lrt -lgcc -lc

# For GumStix compile uncomment the following lines instead:
# routegen_LDFLAGS =
# routegen_MORELIBS =

noinst_PROGRAMS = routegen

routegen_SOURCES = routegen.cpp

routegen_LDADD = \
        ../../src/control/libcontrol.a \
        ../../src/math/libmath.a \
        ../../src/util/libutil.a \
	../../src/python/libpyprops.a \
	@PYTHON_LIBS@

AM_CPPFLAGS = -I$(VPATH)/../../src @PYTHON_INCLUDES@


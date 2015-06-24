# For StarGate compile uncomment the following lines:
# ugear_LDFLAGS = -nodefaultlibs
# ugear_MORELIBS = -lm -lrt -lgcc -lc

# For GumStix compile uncomment the following lines instead:
# routegen_LDFLAGS =
# routegen_MORELIBS =

noinst_PROGRAMS = routegen

routegen_SOURCES = routegen.cpp

routegen_LDADD = \
        $(top_builddir)/src/control/libcontrol.a \
        $(top_builddir)/src/math/libmath.a \
	$(top_builddir)/src/props/libsgprops.a \
        $(top_builddir)/src/util/libutil.a

AM_CPPFLAGS = -I$(VPATH)/../../src


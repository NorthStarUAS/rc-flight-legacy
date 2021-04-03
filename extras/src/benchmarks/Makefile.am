# For StarGate compile uncomment the following lines:
# whetstone_LDFLAGS = -nodefaultlibs
# whetstone_MORELIBS = -lm -lrt -lgcc -lc

# For GumStix compile uncomment the following lines instead:
whetstone =
whetstone_MORELIBS = -lm

noinst_PROGRAMS = spiread whetstone i2c_mcp3427

spiread_SOURCES = \
	spiread.c

i2c_mcp3427_SOURCES = \
	i2c_mcp3427.c

whetstone_SOURCES = \
	whetstone.c

whetstone_LDADD = \
	$(whetstone_MORELIBS)


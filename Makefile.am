EXTRA_DIST = \
	autogen.sh \
	README.gumstix \
	README.verdex \
	README.mpc5200 \
	README.kernel \
	xbow-release.txt

SUBDIRS	= src utils

dist-hook:
	( cd $(top_srcdir); tar --exclude=CVS -cf - data scripts ) \
		| ( cd $(distdir); tar xvf - )
		

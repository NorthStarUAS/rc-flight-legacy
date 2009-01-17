EXTRA_DIST = \
	autogen.sh \
	README.gumstix \
	README.verdex \
	README.kernel \
	xbow-release.txt

SUBDIRS	= src

dist-hook:
	( cd $(top_srcdir); tar --exclude=CVS -cf - data scripts ) \
		| ( cd $(distdir); tar xvf - )
		

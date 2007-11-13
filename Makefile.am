EXTRA_DIST = \
	autogen.sh \
	xbow-release.txt

SUBDIRS	= src

dist-hook:
	( cd $(top_srcdir); tar --exclude=CVS -cf - data scripts ) \
		| ( cd $(distdir); tar xvf - )
		

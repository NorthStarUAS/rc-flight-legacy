main.cxx = *OLD*

dynamichome.cxx = newer gpsd API, but caution, test this carefully on a
greater than 1hz gps ... I was having problems with updates lagging far behind
reality.  I may have inserted a potential fix, but caution it hasn't been
tested and may not be working correctly.

Please see ../autohome/ for a python script that does this job effectively
and has been tested.

#!/usr/bin/python

from line import *
from area import *

# Usage example:
# p1 = Point(0, 1)
# p2 = Point(2, 3)
# p3 = Point(2, 3)
# p4 = Point(0, 4)
p1 = Point(0, 0)
p2 = Point(2, 2)
p3 = Point(0, 2)
p4 = Point(2, 0)

line1 = Line(p1, p2)
line2 = Line(p3, p4)

p = intersect_line_v_line(line1, line2)
if p:
    print "Lines intersect:", p.pretty()
else:
    print "No single intersection point detected"

p = intersect_seg_v_seg(line1, line2)
if p:
    print "Segments intersect:", p.pretty()
else:
    print "No segment intersection detected"

p1 = Point(-3, -4)
p2 = Point(5, -2)
p3 = Point(6, 4)
p4 = Point(-1, 3)

a1 = Area([p1, p2, p3, p4])
print 'area polygon:', a1.pretty()
print 'area center point:', a1.center()

print line1.slope()
print line2.slope()

p = Point(0,2)
print 'dist:', p.pretty(), line1.pretty(), '=', distance_to_line(p, line1)
p = Point(2,0)
print 'dist:', p.pretty(), line1.pretty(), '=', distance_to_line(p, line1)
p = Point(2,2.1)
print 'dist:', p.pretty(), line1.pretty(), '=', distance_to_line(p, line1)

p = Point(0,2)
print 'side:', p.pretty(), line1.pretty(), '=', side_of_line(p, line1)
p = Point(2,0)
print 'side:', p.pretty(), line1.pretty(), '=', side_of_line(p, line1)
p = Point(2,2.1)
print 'side:', p.pretty(), line1.pretty(), '=', side_of_line(p, line1)
p = Point(1,1)
print 'side:', p.pretty(), line1.pretty(), '=', side_of_line(p, line1)

print 'cut:'
advance_dir = Line(Point(0,0), Point(-1,1))
slice(a1, advance_dir)

# with thanks to the contributors at:
#
# https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines-in-python

# this import forces a float result even when dividing two integers
from __future__ import division 
import math

import survey.point as point
import survey.vector
        
class Line():
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
        self.A = (p1.y - p2.y)
        self.B = (p2.x - p1.x)
        self.C = -(p1.x*p2.y - p2.x*p1.y)

    def pretty(self):
        return self.p1.pretty(), self.p2.pretty()

    # return a normalized vector representing the direction of the line
    def direction(self):
        dy = self.p2.y - self.p1.y
        dx = self.p2.x - self.p1.x
        dir = vector.Vector(dx, dy)
        return vector.norm(dir)
    
# compute the intersection of two infinite lines
def intersect_line_vs_line(line1, line2):
    D  = line1.A * line2.B - line1.B * line2.A
    Dx = line1.C * line2.B - line1.B * line2.C
    Dy = line1.A * line2.C - line1.C * line2.A
    if D != 0:
        x = Dx / D
        y = Dy / D
        return point.Point(x,y)
    else:
        return False

# compute the intersetion point of two line segments and check if it
# falls within both segments
def intersect_seg_vs_seg(seg1, seg2):
    p = intersect_line_vs_line(seg1, seg2)
    if p.x < seg1.p1.x and p.x < seg1.p2.x:
        return False
    elif p.x > seg1.p1.x and p.x > seg1.p2.x:
        return False
    elif p.y < seg1.p1.y and p.y < seg1.p2.y:
        return False
    elif p.y > seg1.p1.y and p.y > seg1.p2.y:
        return False
    if p.x < seg2.p1.x and p.x < seg2.p2.x:
        return False
    elif p.x > seg2.p1.x and p.x > seg2.p2.x:
        return False
    elif p.y < seg2.p1.y and p.y < seg2.p2.y:
        return False
    elif p.y > seg2.p1.y and p.y > seg2.p2.y:
        return False
    else:
        return p

# compute the intersetion of an infinite line with a segment and check
# if it falls within the segment
def intersect_line_vs_seg(line1, seg2):
    p = intersect_line_vs_line(line1, seg2)
    if p == False:
        return False
    else:
        print('p (inf):', p.pretty())
        
    if p.x < seg2.p1.x and p.x < seg2.p2.x:
        return False
    elif p.x > seg2.p1.x and p.x > seg2.p2.x:
        return False
    elif p.y < seg2.p1.y and p.y < seg2.p2.y:
        return False
    elif p.y > seg2.p1.y and p.y > seg2.p2.y:
        return False
    else:
        print('p: ', p.pretty())
        return p

# return the sign of x as -1 or 1
def sign(x):
    return math.copysign(1, x)

# project a point orthogonally onto a line.  Return the projected
# point, and the distance along the line relative to the line's
# origin.  Positive distances are in the direction of p1->p2.
def project_point_on_line(p, line):
    # project the point
    dx = line.p2.x - line.p1.x
    dy = line.p2.y - line.p1.y
    d2 = dx*dx + dy*dy
    nx = ((p.x-line.p1.x)*dx + (p.y-line.p1.y)*dy) / d2
    p = point.Point(dx*nx + line.p1.x, dy*nx + line.p1.y)
    # compute the signed distance from the line origin
    distx = p.x - line.p1.x
    disty = p.y - line.p1.y
    if sign(distx) == sign(dx) and sign(disty) == sign(dy):
        dist = math.sqrt(distx*distx + disty*disty)
    else:
        dist = -math.sqrt(distx*distx + disty*disty)
    return p, dist

def distance_to_line(p, line):
    dx = line.p2.x - line.p1.x
    dy = line.p2.y - line.p1.y
    num = abs(dy*p.x - dx*p.y + line.p2.x*line.p1.y - line.p2.y*line.p1.x)
    den = math.sqrt(dy**2 + dx**2)
    if den != 0:
        return num / den
    else:
        return 0

# return if point is left, on, or right of the directed line
def side_of_line(p, line):
    d = (p.y - line.p1.y) * (line.p2.x - line.p1.x) - (p.x - line.p1.x) * (line.p2.y - line.p1.y)
    if d > 0:
        return -1               # left
    elif d < 0:
        return 1                # right
    else:
        return 0                # on the line

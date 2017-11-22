# with thanks to the contributors at:
#
# https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines-in-python

# this import forces a float result even when dividing two integers
from __future__ import division 
import math

from point import *
        
class Line():
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
        self.A = (p1.y - p2.y)
        self.B = (p2.x - p1.x)
        self.C = -(p1.x*p2.y - p2.x*p1.y)

    def pretty(self):
        return self.p1.pretty(), self.p2.pretty()
    
    def slope(self):
        # dy/dx or (y2 - y1) / (x2 - x1)
        dy = self.p2.y - self.p1.y
        dx = self.p2.x - self.p1.x
        if dx != 0:
            return dy / dx
        else:
            return False

# compute the intersection of two infinite lines
def intersect_line_v_line(line1, line2):
    D  = line1.A * line2.B - line1.B * line2.A
    Dx = line1.C * line2.B - line1.B * line2.C
    Dy = line1.A * line2.C - line1.C * line2.A
    if D != 0:
        x = Dx / D
        y = Dy / D
        return Point(x,y)
    else:
        return False

# compute the intersetion point of two line segments and check if it
# falls within both segments
def intersect_seg_v_seg(seg1, seg2):
    p = intersect_lines(seg1, seg2)
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
def intersect_line_v_seg(line1, seg2):
    p = intersect_lines(line1, seg2)
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

def project_point_on_line(p, line):
    dx = line.p2.x - line.p1.x
    dy = line.p2.y - line.p1.y
    d2 = dx*dx + dy*dy
    nx = ((p.x-line.p1.x)*dx + (p.y-line.p1.y)*dy) / d2
    return Point(dx*nx + line.p1.x, dy*nx + line.p1.y)

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

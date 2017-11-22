# this import forces a float result even when dividing two integers
from __future__ import division 

from point import *
from line import *

class Area():
    def __init__(self, points):
        self.points = points

    def pretty(self):
        result = []
        for p in self.points:
            result.append(p.pretty())
        return result
        
    def center(self):
        sumx = 0
        sumy = 0
        for p in self.points:
            sumx += p.x
            sumy += p.y
        return sumx / len(self.points), sumy / len(self.points)

def sign(x):
    return math.copysign(1, x)

# slice an area with a cut line perpendicular to the give dir line and
# advancing along the dir line
def slice(area, dir):
    # project all the area perimiter points onto the advance dir line
    proj = []
    for p in area.points:
        r = project_point_on_line(p, dir)
        print 'orig:', p.pretty(), 'proj:', r.pretty()
        proj.append(r)

    # dir 'vector'
    linex = dir.p2.x - dir.p1.x
    liney = dir.p2.y - dir.p1.y

    # find the point with the most opposite distance with respect to
    # advance direction vector.  This will be the starting point for
    # the slicing operation
    min_index = None
    min_dist = None
    for i, p in enumerate(proj):
        dx = p.x - dir.p1.x
        dy = p.y - dir.p1.y
        if sign(dx) == sign(linex) and sign(dy) == sign(liney):
            dist = math.sqrt(dx*dx + dy*dy)
        else:
            dist = -math.sqrt(dx*dx + dy*dy)
        if min_dist == None or dist < min_dist:
            min_index = i
            min_dist = dist
        print p.pretty(), sign(dx), sign(dy)
    print 'min index:', min_index, 'min dist:', min_dist, area.points[i].pretty(), proj[i].pretty()
        
    # find the slope of the cut line (hint: perpendicular to the
    # advance direction line)
    m = dir.slope()
    if m:
        mi = -1/m
    else:
        mi = 0
    # perpendicular line
    p1 = dir.p1
    p2 = Point(p1.x + 1, p1.y + mi)
    cut = Line(p1, p2)
    print 'cut line:', cut.pretty()
    
    

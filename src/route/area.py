# this import forces a float result even when dividing two integers
from __future__ import division 

import point
from line import *

class Area():
    def __init__(self, points):
        self.points = points
        self.edges = []
        size = len(self.points) - 1
        for i in range(size):
            p1 = self.points[i]
            p2 = self.points[i + 1]
            e = Line(p1, p2)
            self.edges.append(e)
        # close the perimeter
        p1 = self.points[size]
        p2 = self.points[0]
        e = Line(p1, p2)
        self.edges.append(e)
        for e in self.edges:
            print 'edge:', e.pretty()
        
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

    def intersect_with_line(self, line):
        result = []
        size = len(self.edges)
        for i in range(size):
            print 'i:', i, self.edges[i].pretty()
            p = intersect_line_vs_seg(line, self.edges[i])
            if p != False:
                print 'plot:', p.x, p.y
                result.append(p)
        print '  result:', result
        return result

# slice an Area() with a cut line perpendicular to the given dir
# Line() and advancing along the direction Line()
def slice(area, dir, step):
    print 'slice:'
    
    # dir 'vector' (normalized)
    dirx, diry = dir.vector_norm()
    print 'direction vector:', (dirx, diry)

    # cut 'vector' (normalized)
    cutx = -diry
    cuty = dirx
    print 'cut vector:', (cutx, cuty)

    # project all the area points onto the direction line
    proj = []
    dist = []
    for p in area.points:
        r, d = project_point_on_line(p, dir)
        print '   orig:', p.pretty(), 'proj:', r.pretty(), 'dist:', d
        proj.append((r,d))
        dist.append(d)

    for p,d in proj:
        print 'test:', p, d
    
    # find the point with the most opposite distance with respect to
    # advance direction vector.  This will be the starting point for
    # the slicing operation
    min_index = None
    min_dist = None
    for i, (p, d) in enumerate(proj):
        print 'i:', i, 'p:', p, 'd:', d
        if min_dist == None or d < min_dist:
            min_index = i
            min_dist = d
        print '  ', p.pretty()

    print ' start point index:', min_index, 'min dist:', min_dist, area.points[min_index].pretty(), proj[min_index][0].pretty()
    start = point.make_copy( area.points[min_index] )

    # walk through the polygon in 'dir' direction it slicing it with
    # the perpendicular line and find the two extreme point
    # intersections.
    done = False
    while not done:
        # update cut line start point
        start.x += dirx * step
        start.y += diry * step
        print '    ', start.pretty()
        
        # create cut line
        end = point.Point(start.x + cutx, start.y + cuty)
        cut_line = Line(start, end)

        pts = area.intersect_with_line(cut_line)
        if len(pts) < 2:
            done = True
        else:
            pass

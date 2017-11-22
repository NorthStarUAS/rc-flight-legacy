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
            print 'plot:', e.p1.x, e.p1.y
            print 'plot:', e.p2.x, e.p2.y
            print 'plot: '
        
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
                result.append(p)
        print '  result:', result
        return result

# slice an Area() with a cut line perpendicular to the given dir
# Line() and advancing along the direction Line()
def slice(area, dir, step, extend):
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
    for p in area.points:
        r, d = project_point_on_line(p, dir)
        print '   orig:', p.pretty(), 'proj:', r.pretty(), 'dist:', d
        proj.append((r,d))

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
    # update cut line start point
    start.x += dirx * step * 0.5
    start.y += diry * step * 0.5

    # walk through the polygon in 'dir' direction it slicing it with
    # the perpendicular line and find the two extreme point
    # intersections.
    done = False
    toggle = False
    while not done:
        print '    ', start.pretty()
        
        # create cut line
        end = point.Point(start.x + cutx, start.y + cuty)
        cut_line = Line(start, end)

        cut_pts = area.intersect_with_line(cut_line)
        if len(cut_pts) < 2:
            done = True
        else:
            # project all the intersection points onto the cut line.
            # They are already on the cut line, but this function also
            # returns the signed distance along the cut line which is
            # information we need now.
            proj = []
            for p in cut_pts:
                r, d = project_point_on_line(p, cut_line)
                print '   cut:', p.pretty(), 'proj:', r.pretty(), 'dist:', d
                proj.append((r,d))
            # and find the min/max distance points
            min_index = None
            min_dist = None
            max_index = None
            max_dist = None
            for i, (p, d) in enumerate(proj):
                print 'i:', i, 'p:', p, 'd:', d
                if min_dist == None or d < min_dist:
                    min_index = i
                    min_dist = d
                    print '  ', p.pretty()
                if max_dist == None or d > max_dist:
                    max_index = i
                    max_dist = d
                    print '  ', p.pretty()
            minp = cut_pts[min_index]
            maxp = cut_pts[max_index]
            print 'min:', minp.pretty(), 'max:', maxp.pretty()
            if toggle:
                print 'plot:', minp.x - cutx*extend, minp.y - cuty*extend
                print 'plot:', maxp.x + cutx*extend, maxp.y + cuty*extend
            else:
                print 'plot:', maxp.x + cutx*extend, maxp.y + cuty*extend
                print 'plot:', minp.x - cutx*extend, minp.y - cuty*extend
            toggle = not toggle
        # update cut line start point at the end of the loop
        start.x += dirx * step
        start.y += diry * step

        

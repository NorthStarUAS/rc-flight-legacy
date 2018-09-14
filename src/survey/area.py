# this import forces a float result even when dividing two integers
from __future__ import division

import math

from auracore import wgs84

import survey.point as point
import survey.vector as vector
import survey.line as line

d2r = math.pi / 180
r2d = 180 / math.pi

class Area():
    def __init__(self, points):
        self.points = points
        self.edges = []
        size = len(self.points) - 1
        for i in range(size):
            p1 = self.points[i]
            p2 = self.points[i + 1]
            e = line.Line(p1, p2)
            self.edges.append(e)
        # close the perimeter
        p1 = self.points[size]
        p2 = self.points[0]
        e = line.Line(p1, p2)
        self.edges.append(e)
        
    def pretty(self):
        result = []
        for p in self.points:
            result.append(p.pretty())
        return result

    def debug_edges(self):
        for e in self.edges:
            print('edge:', e.pretty())
            print('plot:', e.p1.x, e.p1.y)
            print('plot:', e.p2.x, e.p2.y)
            print('plot: ')

    def center(self):
        sumx = 0
        sumy = 0
        for p in self.points:
            sumx += p.x
            sumy += p.y
        return point.Point( sumx / len(self.points), sumy / len(self.points) )

    def intersect_with_line(self, cut):
        result = []
        size = len(self.edges)
        for i in range(size):
            print('i:', i, self.edges[i].pretty())
            p = line.intersect_line_vs_seg(cut, self.edges[i])
            if p != False:
                result.append(p)
        print('  result:', result)
        return result

# convert list of Point coordinates from geodetic (lon/lat) to cartesian
def geod2cart(ref, geod_points):
    print('geod2cart()')
    result = []
    for p in geod_points:
        # expects points as [lat, lon]
        #heading, dist = gc.course_and_dist( [ref.y, ref.x], [p.y, p.x] )
        (heading, reverse, dist) = \
            wgs84.geo_inverse( ref.y, ref.x, p.y, p.x )
        angle = (90 - heading) * d2r
        x = math.cos(angle) * dist
        y = math.sin(angle) * dist
        print(p.pretty(), 'hdg:', heading, 'dist:', dist, 'cart:', x, y)
        result.append( point.Point(x, y) )
    return result
        
# convert list of Point coordinates from cartesian to geodetic (lon/lat)
def cart2geod(ref, cart_points):
    print('cart2geod()')
    result = []
    for p in cart_points:
        heading = 90 - math.atan2(p.y, p.x) * r2d
        dist = math.sqrt(p.x*p.x + p.y*p.y)
        #lat, lon = gc.project_course_distance( [ref.y, ref.x], heading, dist )
        lat, lon, az2 = wgs84.geo_direct( ref.y, ref.x, heading, dist )
        print(p.pretty(), 'hdg:', heading, 'dist:', dist, 'geod:', lon, lat)
        result.append( point.Point(lon, lat) )
    return result
    
    
# slice an Area() with a cut line perpendicular to the given dir
# Line() and advancing along the direction Line()
def slice(area, dir, step, extend):
    print('slice:')
    
    # dir 'vector' (normalized)
    dir_norm = vector.norm(dir)
    print('direction vector:', dir_norm.pretty())

    # cut 'vector' (normalized)
    cutx = -dir_norm.y
    cuty = dir_norm.x
    print('cut vector:', (cutx, cuty))

    # project all the area points onto the direction line
    proj = []
    adv_line = line.Line( point.Point(0,0), point.Point(dir.x, dir.y) )
    for p in area.points:
        r, d = line.project_point_on_line(p, adv_line)
        print('   orig:', p.pretty(), 'proj:', r.pretty(), 'dist:', d)
        proj.append((r,d))

    for p,d in proj:
        print('test:', p, d)
    
    # find the point with the most opposite distance with respect to
    # advance direction vector.  This will be the starting point for
    # the slicing operation
    min_index = None
    min_dist = None
    for i, (p, d) in enumerate(proj):
        print('i:', i, 'p:', p, 'd:', d)
        if min_dist == None or d < min_dist:
            min_index = i
            min_dist = d
        print('  ', p.pretty())

    print(' start point index:', min_index, 'min dist:', min_dist, area.points[min_index].pretty(), proj[min_index][0].pretty())
    start = point.Point( area.points[min_index].x, area.points[min_index].y )
    # update cut line start point
    start.x += dir_norm.x * step * 0.5
    start.y += dir_norm.y * step * 0.5

    # walk through the polygon in 'dir' direction it slicing it with
    # the perpendicular line and find the two extreme point
    # intersections.
    done = False
    slices = []
    while not done:
        print('    ', start.pretty())
        
        # create cut line
        end = point.Point(start.x + cutx, start.y + cuty)
        cut_line = line.Line(start, end)

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
                r, d = line.project_point_on_line(p, cut_line)
                print('   cut:', p.pretty(), 'proj:', r.pretty(), 'dist:', d)
                proj.append((r,d))
            # and find the min/max distance points
            min_index = None
            min_dist = None
            max_index = None
            max_dist = None
            for i, (p, d) in enumerate(proj):
                print('i:', i, 'p:', p, 'd:', d)
                if min_dist == None or d < min_dist:
                    min_index = i
                    min_dist = d
                    print('  ', p.pretty())
                if max_dist == None or d > max_dist:
                    max_index = i
                    max_dist = d
                    print('  ', p.pretty())
            minp = cut_pts[min_index]
            maxp = cut_pts[max_index]
            print('min:', minp.pretty(), 'max:', maxp.pretty())
            slices.append( (point.Point(start.x, start.y), min_dist, max_dist) )
        # update cut line start point at the end of the loop
        start.x += dir_norm.x * step
        start.y += dir_norm.y * step

    size = len(slices)
    i = 0
    toggle = True
    route = []
    while i < size:
        print('i:', i)
        if i > 0:
            (s0, min0, max0) = slices[i-1]
        else:
            (s0, min0, max0) = slices[i]
        (s1, min1, max1) = slices[i]
        if i < size - 1:
            (s2, min2, max2) = slices[i+1]
        else:
            (s2, min2, max2) = slices[i]
        min = min1
        if  toggle:
            if min0 < min: min = min0
        else:
            if min2 < min: min = min2
        max = max1
        if not toggle:
            if max0 > max: max = max0
        else:
            if max2 > max: max = max2
        p1 = point.Point(s1.x + cutx*(min - extend),
                         s1.y + cuty*(min - extend))
        p2 = point.Point(s1.x + cutx*(max + extend),
                         s1.y + cuty*(max + extend))
        if toggle:
            print('plot:', p1.x, p1.y)
            print('plot:', p2.x, p2.y)
            route.append(p1)
            route.append(p2)
        else:
            print('plot:', p2.x, p2.y)
            print('plot:', p1.x, p1.y)
            route.append(p2)
            route.append(p1)
        i += 1
        toggle = not toggle

    # and finally return the result
    return route

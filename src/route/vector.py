# an simple vector class

import math

import point

class Vector(point.Point):
    pass

# return a normalized vector
def norm(v):
    dist = math.sqrt(v.x*v.x + v.y*v.y)
    return Vector(v.x/dist, v.y/dist)

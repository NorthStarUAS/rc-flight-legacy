# an simple vector class

import math

import survey.point as point

class Vector(point.Point):
    pass

# return the length of a vector
def length(v):
    return math.sqrt(v.x*v.x + v.y*v.y)

# return a normalized vector
def norm(v):
    dist = length(v)
    return Vector(v.x/dist, v.y/dist)


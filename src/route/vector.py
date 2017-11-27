# an simple vector class

import math

class Vector():
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def pretty(self):
        return (self.x, self.y)

# return a normalized vector
def norm(v):
    dist = math.sqrt(v.x*v.x + v.y*v.y)
    return Vector(v.x/dist, v.y/dist)

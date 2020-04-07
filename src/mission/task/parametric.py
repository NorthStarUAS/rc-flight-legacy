# This is a prototype task designed to follow a parametric equation.  It uses
# an optimizer to find the closest "t" to the current location, then computes
# curvature at that point and updates the circle hold parameters to match.  So
# in the end it's the circle hold controller that is active, but this module
# is continuously updating the circle parameters.

# Currently:
# The parametric equation is hard coded.
# The scale is hard coded
# The orientation is hard coded.
# The center point is 'home'

import math
import numpy as np
# from scipy.optimize import minimize

from props import getNode
from auracore import wgs84

import comms.events
from mission.task.task import Task
import mission.task.state

# various support code and values for parameterized functions

# loop distance (may need to be adjusted for each specific parametric function)
loop_t = math.pi * 2
r2d = 180/math.pi
half_width = 250
half_height = 75
vertical = 50

# parameterized functions for x and y
loop_t = math.pi * 2
def simple_func(t):
    x = math.sin(t)
    y = math.sin(2*t)
    return ( half_width * x, half_height * y )

# Lemniscate of Bernoulli
# https://en.wikipedia.org/wiki/Lemniscate_of_Bernoulli
sqrt2 = math.sqrt(2)
def lemniscate(t):
    sint = math.sin(t)
    cost = math.cos(t)
    x = cost / (sint * sint + 1.0)
    y = 2 * sqrt2 * cost*sint / (sint * sint + 1.0)
    return ( half_width * x, half_height * y )

# Rhodonea (Rose) function
# https://en.wikipedia.org/wiki/Rose_(mathematics)
# k = petal coefficient
# if k is odd then k is the number of petals
# if k is even then k is half the number of petals
# if k is a half-integer (e.g. 1/2, 3/2, 5/2), the curve will be
#    rose-shaped with 4k petals.
# see the web link for more details on k
loopt = 2*math.pi
def rose(t):
    k = 2
    x = math.cos(k*t) * math.cos(t)
    y = math.cos(k*t) * math.sin(t)
    return ( half_width * x, half_width * y )
    
#func = simple_func
#func = lemniscate
func = rose

# distance from specified point to value of curve at 't'
def distance_t2xy(t, x, y):
    (fx, fy) = func(t)
    dx = x - fx
    dy = y - fy
    return dx*dx + dy*dy

# distance 2d distance between two t values
def distance_t2t(t1, t2):
    (x1, y1) = func(t1)
    (x2, y2) = func(t2)
    dx = x2 - x1; dy = y2 - y1
    return math.sqrt(dx*dx + dy*dy)

# return an estimate of the tangent heading at point t
def tangent_at_t(t, step):
    t1 = t - step
    t2 = t + step
    (x1, y1) = func(t1)
    (x2, y2) = func(t2)
    dx = x2 - x1; dy = y2 - y1
    return math.pi*0.5 - math.atan2(dy, dx)
    
def define_circle(p1, p2, p3):
    """
    Returns the center and radius of the circle passing the given 3 points.
    In case the 3 points form a line, returns (None, infinity).
    """
    temp = p2[0] * p2[0] + p2[1] * p2[1]
    bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
    cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
    det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

    if abs(det) < 1.0e-6:
        return (None, np.inf, "none")
    elif det < 0.0:
        direction = "right"
    else:
        direction = "left"

    # Center of circle
    cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
    cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

    radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)
    return ((cx, cy), radius, direction)

# return an estimate of the radius of curvature at point t
def curvature_at_t(t, step):
    t1 = t - step
    t2 = t + step
    p1 = func(t1)
    p2 = func(t)
    p3 = func(t2)
    return define_circle(p1, p2, p3)
    
# optimization function: find the value of t that corresponds to the
# point on the curve that is closest to the to the given x, y
# coordinates.
# def find_best_t(x, y, initial_guess):
#     x0 = [ initial_guess ]
#     bnds = [ (initial_guess, None) ]
#     res = minimize(distance_t2xy, x0, bounds=bnds, args=(x, y), tol=0.01,
#                    options={'disp': False})
#     # print(res)
#     return res.x[0]

def find_next_t(x, y, initial_guess, dt):
    t = initial_guess
    d = distance_t2xy(t, x, y)
    while True:
        min_dist = d
        min_t = t
        t += dt
        d = distance_t2xy(t, x, y)
        if d > min_dist:
            break
    return min_t


class Parametric(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.pos_node = getNode("/position", True)
        self.home_node = getNode("/task/home", True)
        self.circle_node = getNode("/task/circle/active", True)
        self.ap_node = getNode("/autopilot", True)
        self.targets_node = getNode("/autopilot/targets", True)
        self.nav_node = getNode("/navigation", True)
        self.tecs_node = getNode("/config/autopilot/TECS", True)

        self.t = 0.0
        self.direction = "left"
        self.radius_m = 100.0
        
        self.name = config_node.getString("name")
        if config_node.hasChild("direction"):
            # only override the default if a value is given
            self.direction = config_node.getString("direction")
        if config_node.hasChild("radius_m"):
            # only override the default if a value is given
            self.radius_m = config_node.getFloat("radius_m")
            
        # estimate the distance of the route (assuming loop_t is set correctly!)
        steps = 1000
        self.step = loop_t / float(steps)
        t1 = 0
        t2 = t1 + self.step
        self.loop_dist = 0.0
        while t2 <= loop_t + 0.5*self.step:
            self.loop_dist += distance_t2t(t1, t2)
            t1 = t2
            t2 += self.step
        print('route distance is approximately:', self.loop_dist)

    def activate(self):
        self.active = True

        # save existing state
        mission.task.state.save(modes=True, circle=True, targets=True)
        
        # set modes
        self.ap_node.setString("mode", "basic+tecs")
        self.nav_node.setString("mode", "circle")
        comms.events.log("mission", "parametric path")
    
    def update(self, dt):
        if not self.active:
            return False

        # home manager provides current position in a mini 2d
        # cartesian coordinate system relative to home.
        x_m = self.home_node.getFloat("x_m")
        y_m = self.home_node.getFloat("y_m")

        #self.t = find_best_t(x_m, y_m, initial_guess=self.t)
        self.t = find_next_t(x_m, y_m, initial_guess=self.t,
                             dt=1.0/self.loop_dist)
        if self.t > loop_t:
            self.t -= loop_t

        (center, radius, direction) = curvature_at_t(self.t, self.step)
        #print('t:', self.t, ' radius: ', radius, direction)

        if center != None:
            # project center back to lat, lon
            offset_dist = math.sqrt(center[0]*center[0] + center[1]*center[1])
            circle_offset_deg = 90 - math.atan2(center[1], center[0]) * r2d
            (cc_lat, cc_lon, az2) = \
                wgs84.geo_direct( self.home_node.getFloat("latitude_deg"),
                                  self.home_node.getFloat("longitude_deg"),
                                  circle_offset_deg, offset_dist )
            self.circle_node.setFloat('latitude_deg', cc_lat)
            self.circle_node.setFloat('longitude_deg', cc_lon)
            self.circle_node.setString('direction', direction)
            self.circle_node.setFloat('radius_m', radius)

            # adjust target altitude to be swoopy
            x = self.home_node.getFloat('dist_m')
            factor = math.sqrt(vertical) / half_width
            h = factor*factor * x*x
            self.targets_node.setFloat('altitude_agl_ft', 200 + h)

            # adjust target airspeed to be swoopy
            min = self.tecs_node.getFloat("min_kt")
            max = self.tecs_node.getFloat("max_kt")
            range = max - min
            v = range / (half_width*half_width) * x*x
            airspeed_kt = max - v
            if airspeed_kt < min:
                airspeed_kt = min
            self.targets_node.setFloat("airspeed_kt", airspeed_kt)
            
    def is_complete(self):
        return False
    
    def close(self):
        # restore the previous state
        mission.task.state.restore()

        self.active = False
        return True

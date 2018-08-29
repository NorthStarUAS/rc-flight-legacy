# high level navigation modes

from props import getNode

import circle
import route

nav_node = getNode("/navigation", True)

def init():
    circle.init()
    route.init()

def update(dt):
    if nav_node.getString("mode") == 'circle':
        circle.update(dt)
    elif nav_node.getString("mode") == 'route':
        route.update(dt)
    # hack test (override target roll)
    #targets_node = getNode("/autopilot/targets", True)
    #hack_roll = targets_node.getFloat("hack_roll_deg")
    #targets_node.setFloat("roll_deg", hack_roll)
    

# high level navigation modes

from props import root, getNode

import circle

nav_node = getNode("/navigation", True)

def init():
    circle.init()

def update(dt):
    if nav_node.getString("mode") == 'circle':
        circle.update(dt)

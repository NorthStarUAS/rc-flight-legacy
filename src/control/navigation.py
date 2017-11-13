# high level navigation modes

from props import getNode

import circle
import route

nav_node = getNode("/navigation", True)

def init():
    nav_node['mode'] = ''
    circle.init()
    route.init()

def update(dt):
    if nav_node['mode'] == 'circle':
        circle.update(dt)
    elif nav_node['mode'] == 'route':
        route.update(dt)

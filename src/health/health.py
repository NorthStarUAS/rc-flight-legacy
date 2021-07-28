# system health module

# Note: this is extremely simple right now (simply polling os load
# average).  The intent is to include more system sanity checking in
# this module.  Currently there are various random sanity checks
# scattered throughout the code where it makes sense to put them
# closer to their data source or usage.

import os
from PropertyTree import PropertyNode

def init():
    global status_node
    status_node = PropertyNode("/status")

def update():
    load = os.getloadavg()
    status_node.setFloat("system_load_avg", load[0])

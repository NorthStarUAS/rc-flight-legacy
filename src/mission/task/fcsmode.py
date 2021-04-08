# mode manager: translate high level 'mode' requests to the corresponding
# sets of enable flags

import re

from props import getNode

import comms.events

ap_node = getNode("/autopilot", True)
locks_node = getNode("/autopilot/locks", True)

def get():
    return ap_node.getString("mode")

# manage detailed enable switches from high level mode
def set(fcs_mode):
    ap_node.setString("mode", fcs_mode)
    comms.events.log('control', 'mode change: ' + fcs_mode)
    if fcs_mode == 'basic':
        # set lock modes for 'basic' inner loops only
        locks_node.setBool( 'roll', True )
        locks_node.setBool( 'yaw', True )
        locks_node.setBool( 'pitch', True )
        locks_node.setBool( 'tecs', False )
    elif fcs_mode == 'roll':
        # set lock modes for roll only
        locks_node.setBool( 'roll', True )
        locks_node.setBool( 'yaw', False )
        locks_node.setBool( 'pitch', False )
        locks_node.setBool( 'tecs', False )
    elif fcs_mode == 'roll+pitch':
        # set lock modes for roll and pitch
        locks_node.setBool( 'roll', True )
        locks_node.setBool( 'yaw', False )
        locks_node.setBool( 'pitch', True )
        locks_node.setBool( 'tecs', False )
    elif fcs_mode == 'basic+tecs':
        # set lock modes for 'basic' + alt hold + speed hold
        locks_node.setBool( 'roll', True )
        locks_node.setBool( 'yaw', True )
        locks_node.setBool( 'pitch', True )
        locks_node.setBool( 'tecs', True )
    else:
        comms.events.log("control", "unknown fcs mode attempted: " + fcs_mode)

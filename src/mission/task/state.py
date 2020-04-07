# State stack: save and restore key state variables using a stack metaphore.
# this is used by tasks that wish change system state when they are activated
# and restore the previous state when they complete.

from props import getNode

ap_node = getNode('/autopilot', True)
nav_node = getNode('/navigation', True)
targets_node = getNode('/autopilot/targets', True)
circle_node = getNode('/task/circle/active', True)

class State():
    # copy a snapshot of the current state
    def __init__(self, save_modes, save_circle, save_targets):
        self.modes = {}
        if save_modes:
            self.modes['fcs'] = ap_node.getString('mode')
            self.modes['nav'] = nav_node.getString('mode')
        
        self.circle = {}
        if save_circle:
            self.circle['lon_deg'] = circle_node.getFloat("longitude_deg")
            self.circle['lat_deg'] = circle_node.getFloat("latitude_deg")
            self.circle['direction'] = circle_node.getString("direction")
            self.circle['radius_m'] = circle_node.getFloat("radius_m")
            
        self.targets = {}
        if save_targets:
            self.targets['agl_ft'] = targets_node.getFloat("altitude_agl_ft")
            self.targets['speed_kt'] = targets_node.getFloat("airspeed_kt")

    # restore the saved state to the running system
    def restore():
        if len(self.modes):
            ap_node.setString("mode", self.modes['fcs'])
            nav_node.setString("mode", self.modes['nav'])
        if len(self.circle):
            circle_node.setFloat("longitude_deg", self.circle['lon_deg'])
            circle_node.setFloat("latitude_deg", self.circle['lat_deg'])
            circle_node.setString("direction", self.circle['direction'])
            circle_node.setFloat("radius_m", self.circle['radius_m'])
        if len(self.targets):
            targets_node.setFloat("altitude_agl_ft", self.targets['agl_ft'])
            targets_node.setFloat("airspeed_kt", self.targets['speed_kt'])


# define the stack

stack = []

def save(modes=False, circle=False, targets=False):
    new_state = State(modes, circle, targets)
    stack.append(new_state)

def restore():
    saved_state = stack.pop()
    saved_state.restore()
    

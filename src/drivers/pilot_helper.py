# pilot_helper.py

from props import getNode

from comms import events

class pilot_helper():
    def __init__(self):
        self.last_fail_safe = False
        pass
    
    def init(self):
        self.pilot_node = getNode("/sensors/pilot_input", True)
        self.flight_node = getNode("/controls/flight", True)
        self.engine_node = getNode("/controls/engine", True)
        self.ap_node = getNode("/autopilot", True)

    def update(self):
        # log receiver fail safe changes
        if self.pilot_node.getBool("fail_safe") != self.last_fail_safe:
            msg = "Receiver fail safe = %d" % self.pilot_node.getBool("fail_safe")
            events.log("Aura", msg );
            self.last_fail_safe = self.pilot_node.getBool("fail_safe")

        # Only in manual mode, do copy the pilot inputs to the main AP
        # outputs.  This puts the pilot inputs in a standard place and
        # allows the AP to seed it's components with trimmed values
        # and improve continuity when switching from manual to AP
        # mode.
        if not self.ap_node.getBool("master_switch"):
            self.flight_node.setFloat("aileron", self.pilot_node.getFloat("aileron"))
            self.flight_node.setFloat("elevator", self.pilot_node.getFloat("elevator"))
            self.engine_node.setFloat("throttle", self.pilot_node.getFloat("throttle"))
            self.flight_node.setFloat("rudder", self.pilot_node.getFloat("rudder"))
            self.flight_node.setFloat("flaps", self.pilot_node.getFloat("flaps"))

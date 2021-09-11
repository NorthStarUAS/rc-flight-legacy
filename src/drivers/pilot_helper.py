# pilot_helper.py

from PropertyTree import PropertyNode

from comms import events

class pilot_helper():
    def __init__(self):
        self.last_fail_safe = False
        pass
    
    def init(self):
        self.pilot_node = PropertyNode("/sensors/pilot_input")
        self.flight_node = PropertyNode("/controls/flight")
        self.engine_node = PropertyNode("/controls/engine")
        self.switches_node = PropertyNode("/autopilot")

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
        if not self.switches_node.getBool("master_switch"):
            self.flight_node.setDouble("aileron", self.pilot_node.getDouble("aileron"))
            self.flight_node.setDouble("elevator", self.pilot_node.getDouble("elevator"))
            self.engine_node.setDouble("throttle", self.pilot_node.getDouble("throttle"))
            self.flight_node.setDouble("rudder", self.pilot_node.getDouble("rudder"))
            self.flight_node.setDouble("flaps", self.pilot_node.getDouble("flaps"))

from props import getNode

import comms.events
from mission.task.task import Task
import mission.task.state

class Preflight(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.task_node = getNode("/task", True)
        self.preflight_node = getNode("/task/preflight", True)
        self.ap_node = getNode("/autopilot", True)
        self.targets_node = getNode("/autopilot/targets", True)
        self.imu_node = getNode("/sensors/imu", True)
        self.flight_node = getNode("/controls/flight", True)
        #self.saved_fcs_mode = ""
        self.timer = 0.0
        self.duration_sec = 60.0
        self.name = config_node.getString("name")

        # copy to /task/preflight
        if config_node.hasChild("duration_sec"):
            self.duration_sec = config_node.getFloat("duration_sec")
        self.preflight_node.setFloat("duration_sec", self.duration_sec)

    def activate(self):
        self.active = True

        # save existing state
        mission.task.state.save(modes=True)

        if not self.task_node.getBool("is_airborne"):
            # set fcs mode to roll+pitch, aka vanity mode? :-)
            self.ap_node.setString("mode", "roll+pitch")
            self.targets_node.setFloat("roll_deg", 0.0)
            self.targets_node.setFloat("pitch_deg", 0.0)
            self.flight_node.setFloat("flaps_setpoint", 0.0)
            # reset timer
            self.timer = 0.0
        else:
            # we are airborne, don't change modes and configure timer
            # to be already expired
            self.timer = self.preflight_node.getFloat("duration_sec") + 1.0
        comms.events.log("mission", "preflight")

    def update(self, dt):
        if not self.active:
            return False
        # print "preflight & updating"
        self.timer += dt

    def is_complete(self):
        # print "timer=%.1f duration=%.1f" % (self.timer, self.duration_sec)
        # complete when timer expires or we sense we are airborne
        # (sanity check!)
        done = False
        if self.timer >= self.preflight_node.getFloat("duration_sec") or \
           self.task_node.getBool("is_airborne"):
            done = True
        return done

    def close(self):
        # restore the previous state
        mission.task.state.restore()
        
        self.active = False
        return True

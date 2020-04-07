from props import getNode

import comms.events
from mission.task.task import Task

class Calibrate(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.sensors_node = getNode("/sensors", True)
        self.gps_node = getNode("/sensors/gps", True)
        self.home_node = getNode("/task/home", True)
        self.task_node = getNode("/task", True)
        self.completed = False
        self.name = config_node.getString("name")

    def activate(self):
        self.active = True
        self.completed = False
        comms.events.log("mission", "calibrate task")
    
    def update(self, dt):
        if not self.active:
            return False

        if self.task_node.getBool("is_airborne"):
            # do nothing if we are airborne
            pass
        else:
            comms.events.log("mission", "reset home position, ground elevation, zero airspeed.")
            # home position
            if self.gps_node.getBool("settle"):
                # reset home position if gps is settled (and thus
                # valid for some amount of time.)  This is
                # accomplished by marking home as not set, and then
                # allowing the home_mgr task to set a new home
                # position as a side effect.
                self.home_node.setBool("valid", False)

            # calibrate ground elevation: this should settle in
            # about 30 seconds if we just leave the plane sit ...

            # airspeed
            self.sensors_node.setBool("airdata_calibrate", True)

        # run once and then we are done.  The completed flag is set to
        # false when the task is activated so this task can be pushed
        # on the active queue any number of times by external user
        # request.
        self.completed = True

    def is_complete(self):
        return self.completed
    
    def close(self):
        self.active = False
        return True

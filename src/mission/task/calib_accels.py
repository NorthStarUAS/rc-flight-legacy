from props import getNode

import comms.events
from mission.task.task import Task
from mission.task.lowpass import LowPass

# state key:
#   0 = not started
#   1 = rightside up
#   2 = upside down
#   3 = nose down
#   4 = nose up
#   5 = right wing down
#   6 = right wing up
#   7 = complete ok
#   8 = complete failed

class CalibrateAccels(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.imu_node = getNode("/sensors/imu", True)
        self.state = 0
        self.ax_slow = LowPass(time_factor=2.0) 
        self.ax_fast = LowPass(time_factor=0.2) 
        self.ay_slow = LowPass(time_factor=2.0) 
        self.ay_fast = LowPass(time_factor=0.2) 
        self.az_slow = LowPass(time_factor=2.0) 
        self.az_fast = LowPass(time_factor=0.2)
        self.armed = False

    def activate(self):
        self.active = True
        comms.events.log("calibrate accels", "active")

    def detect_up(self, ax, ay, az):
        if ax > 8: return "x-pos"    # nose up
        elif ax < -8: return "x-neg" # nose down
        if ay > 8: return "y-pos"    # right wing down
        elif ay < -8: return "y-neg" # right wing up
        if az > 8: return "z-pos"    # up side down
        elif az < -8: return "z-neg" # right side up
        return "none"                # no dominate axis up

    def update(self, dt):
        if not self.active:
            return False

        # update filters
        ax = self.imu_node.getFloat("ax_nocal")
        ay = self.imu_node.getFloat("ay_nocal")
        az = self.imu_node.getFloat("az_nocal")
        self.ax_slow.update(ax, dt)
        self.ax_fast.update(ax, dt)
        self.ay_slow.update(ay, dt)
        self.ay_fast.update(ay, dt)
        self.az_slow.update(az, dt)
        self.az_fast.update(az, dt)

        # (no) motion test
        ax_diff = self.ax_slow.filter_value - self.ax_fast.filter_value
        ay_diff = self.ay_slow.filter_value - self.ay_fast.filter_value
        az_diff = self.az_slow.filter_value - self.az_fast.filter_value
        if abs(ax_diff) < 0.02 and abs(ay_diff) < 0.02 and abs(az_diff) < 0.02:
            stable = True
        else:
            stable = False
            
        dir = self.detect_up(self.ax_fast.filter_value,
                             self.ay_fast.filter_value,
                             self.az_fast.filter_value)
        if dir == "none":
            self.armed = True
        print("orient:", dir, "stable:", stable, "armed:", self.armed,
              "slow-fast: %.3f %.3f %.3f" % (ax_diff, ay_diff, az_diff))
              
        if self.state == 0:
            # opportunity to initialize stuff
            self.state += 1
            self.armed = False
        elif self.state == 1:
            print("Place level and right side up - stable:", stable)
            if self.armed and stable:
                # take measurement
                self.state += 1
                self.armed = False
        elif self.state == 2:
            print("Place up side down - stable:", stable)
            if self.armed and stable:
                # take measurement
                self.state += 1
                self.armed = False
        elif self.state == 3:
            print("Place nose down - stable:", stable)
            if self.armed and stable:
                # take measurement
                self.state += 1
                self.armed = False
        elif self.state == 4:
            print("Place nose up - stable:", stable)
            if self.armed and stable:
                # take measurement
                self.state += 1
                self.armed = False
        elif self.state == 5:
            print("Place right wing up - stable:", stable)
            if self.armed and stable:
                # take measurement
                self.state += 1
                self.armed = False
        elif self.state == 6:
            print("Place right wing down - stable:", stable)
            if self.armed and stable:
                # take measurement
                self.state += 1
                self.armed = False
        
    def is_complete(self):
        return False

    def close(self):
        self.active = False
        return True

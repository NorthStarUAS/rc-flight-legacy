import math
import numpy as np
import os

from props import getNode, PropertyNode
import props_json

import comms.events
from mission.task.task import Task
from mission.task.lowpass import LowPass
import mission.task.transformations as tr

g = 9.81                        # gravity

# state key:
#   0 = right side up
#   1 = up side down
#   2 = nose down
#   3 = nose up
#   4 = right wing up
#   5 = right wing down
#   6 = sanity check
#   7 = complete ok
#   8 = complete failed

class CalibrateAccels(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.name = config_node.getString("name")
        self.imu_node = getNode("/sensors/imu", True)
        self.config_imu_node = getNode("/config/drivers/Aura4/imu", True)
        self.task_node = getNode("/task", True)
        self.state = 0
        self.ax_slow = LowPass(time_factor=1.0) 
        self.ax_fast = LowPass(time_factor=0.2) 
        self.ay_slow = LowPass(time_factor=1.0) 
        self.ay_fast = LowPass(time_factor=0.2) 
        self.az_slow = LowPass(time_factor=1.0) 
        self.az_fast = LowPass(time_factor=0.2)
        self.armed = False
        self.ref = [ [  0,  0, -g ],
                     [  0,  0,  g ],
                     [ -g,  0,  0 ],
                     [  g,  0,  0 ],
                     [  0, -g,  0 ],
                     [  0,  g,  0 ] ]
        self.meas = list(self.ref) # copy
        self.checked = {}
        
    def activate(self):
        self.active = True
        self.armed = False
        self.checked = {}
        comms.events.log("calibrate accels", "active")

    def detect_up(self):
        ax = self.ax_fast.filter_value
        ay = self.ay_fast.filter_value
        az = self.az_fast.filter_value
        if ax > 8: return "x-pos"    # nose up
        elif ax < -8: return "x-neg" # nose down
        if ay > 8: return "y-pos"    # right wing down
        elif ay < -8: return "y-neg" # right wing up
        if az > 8: return "z-pos"    # up side down
        elif az < -8: return "z-neg" # right side up
        return "none"                # no dominate axis up

    def new_axis(self):
        up_axis = self.detect_up()
        if up_axis == "none":
            return False
        elif up_axis in self.checked:
            return False
        else:
            return True
        
    def update(self, dt):
        if not self.active:
            return False

        # update filters
        ax = self.imu_node.getFloat("ax_raw")
        ay = self.imu_node.getFloat("ay_raw")
        az = self.imu_node.getFloat("az_raw")
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
        d = math.sqrt(ax_diff*ax_diff + ay_diff*ay_diff + az_diff*az_diff)
        if d < 0.04:
            stable = True
        else:
            stable = False
            
        up_axis = self.detect_up()
        if up_axis == "none":
            self.armed = True
        if self.state < 6:
            print("up axis:", up_axis, "armed:", self.armed, " slow-fast: %.3f" % d, " stable:", stable)

        if not self.armed:
            self.task_node.setInt("calib_state", 99)
        else:
            self.task_node.setInt("calib_state", self.state)
            
        if self.state == 0:
            print("Place level and right side up - stable:", stable)
            if self.armed and stable and self.new_axis():
                self.meas[self.state] = [ self.ax_fast.filter_value,
                                          self.ay_fast.filter_value,
                                          self.az_fast.filter_value ]
                self.checked[up_axis] = True
                self.state += 1
                self.armed = False
        elif self.state == 1:
            print("Place up side down - stable:", stable)
            if self.armed and stable and self.new_axis():
                self.meas[self.state] = [ self.ax_fast.filter_value,
                                          self.ay_fast.filter_value,
                                          self.az_fast.filter_value ]
                self.checked[up_axis] = True
                self.state += 1
                self.armed = False
        elif self.state == 2:
            print("Place nose down - stable:", stable)
            if self.armed and stable and self.new_axis():
                self.meas[self.state] = [ self.ax_fast.filter_value,
                                          self.ay_fast.filter_value,
                                          self.az_fast.filter_value ]
                self.checked[up_axis] = True
                self.state += 1
                self.armed = False
        elif self.state == 3:
            print("Place nose up - stable:", stable)
            if self.armed and stable and self.new_axis():
                self.meas[self.state] = [ self.ax_fast.filter_value,
                                          self.ay_fast.filter_value,
                                          self.az_fast.filter_value ]
                self.checked[up_axis] = True
                self.state += 1
                self.armed = False
        elif self.state == 4:
            print("Place right wing down - stable:", stable)
            if self.armed and stable and self.new_axis():
                self.meas[self.state] = [ self.ax_fast.filter_value,
                                          self.ay_fast.filter_value,
                                          self.az_fast.filter_value ]
                self.checked[up_axis] = True
                self.state += 1
                self.armed = False
        elif self.state == 5:
            print("Place right wing up - stable:", stable)
            if self.armed and stable and self.new_axis():
                self.meas[self.state] = [ self.ax_fast.filter_value,
                                          self.ay_fast.filter_value,
                                          self.az_fast.filter_value ]
                self.checked[up_axis] = True
                self.state += 1
                self.armed = False
        elif self.state == 6:
            # did we measure 6 unique axes?
            if len(self.checked) != 6:
                print("Somehow didn't calibrate 6 orientations. :-(")
                self.state += 2
            else:
                # compute affine rotation fit
                v0 = np.array(self.meas, dtype=np.float64, copy=True).T
                v1 = np.array(self.ref, dtype=np.float64, copy=True).T
                self.accel_affine = tr.affine_matrix_from_points(v0, v1, shear=True, scale=True)
                print("accel_affine:\n", self.accel_affine)
                self.scale, shear, angles, self.translate, perspective = tr.decompose_matrix(self.accel_affine)
                print("scale:", self.scale)
                print("shear:", shear)
                print("angles:", angles)
                print("translate:", self.translate)
                print("perspective:", perspective)

                # recompose the original affine matrix with:
                # translate @ rotate @ scale
                T = tr.translation_matrix(self.translate)
                self.R = tr.euler_matrix(*angles)
                S = np.diag([self.scale[0], self.scale[1], self.scale[2], 1.0])
                print("T:\n", T)
                print("R:\n", self.R)
                print("S:\n", S)
                print("R @ R.T:\n", self.R @ self.R.T)
                recompose = T @ self.R @ S
                print("recompose:\n", recompose)
                # check rotation matrix, if any row or column doesn't
                # have an element close to 1, then bomb
                if np.max(np.abs(self.R[0])) < 0.9:
                    print("bad row 1")
                    self.state += 2
                elif np.max(np.abs(self.R[1])) < 0.9:
                    print("bad row 2")
                    self.state += 2
                elif np.max(np.abs(self.R[2])) < 0.9:
                    print("bad row 3")
                    self.state += 2
                elif np.max(np.abs(self.R[:,0])) < 0.9:
                    print("bad column 1")
                    self.state += 2
                elif np.max(np.abs(self.R[:,1])) < 0.9:
                    print("bad column 2")
                    self.state += 2
                elif np.max(np.abs(self.R[:,2])) < 0.9:
                    print("bad column 3")
                    self.state += 2
                else:
                    # nothing bad detected, goto success state
                    self.state += 1
        elif self.state == 7:
            # calibration complete, success, report!
            comms.events.log("calibrate accels", "calibration succeeded.")
            #print("strapdown calibration:")
            #print(self.R)
            # as if this wasn't already fancy enough, get even fancier!
            errors = []
            for i, v in enumerate(self.meas):
                print("measure:", i, v)
                v1 =  self.accel_affine @ np.hstack((v, 1))
                v0 = self.ref[i]
                err = np.linalg.norm(v0 - v1[:3])
                errors.append(err)
            #print("errors:", errors)
            mean = np.mean(errors)
            std = np.std(errors)
            #print("calibration mean:", mean, " std:", std)
            self.state += 2
            calib_node = self.config_imu_node.getChild("calibration", True)
            calib_node.setLen("strapdown", 9)
            for i in range(9):
                calib_node.setFloatEnum("strapdown", i, self.R[:3,:3].flatten()[i])
            calib_node.setFloat("accel_fit_mean", mean)
            calib_node.setFloat("accel_fit_std", std)
            calib_node.setLen("accel_scale", 3)
            for i in range(3):
                calib_node.setFloatEnum("accel_scale", i, self.scale[i])
            calib_node.setLen("accel_translate", 3)
            for i in range(3):
                calib_node.setFloatEnum("accel_translate", i, self.translate[i])
            home = os.path.expanduser("~")
            filename = os.path.join(home, "imu_calibration.json")
            props_json.save(filename, calib_node)
            comms.events.log("calibrate accels", "saved results to: " + filename)
        elif self.state == 8:
            # calibration complete, but failed. :-(
            comms.events.log("calibrate accels", "calibration FAILED!")
            pass            

    def is_complete(self):
        return False

    def close(self):
        self.active = False
        return True

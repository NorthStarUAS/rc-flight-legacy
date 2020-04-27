import math
import numpy as np
import os

from props import getNode, PropertyNode
import props_json

import comms.events
from mission.task.task import Task
from mission.task.lowpass import LowPass
from mission.task.matutil import affine_matrix_from_points, decompose_matrix

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
        self.imu_node = getNode("/sensors/imu", True)
        self.config_imu_node = getNode("/config/drivers/Aura4/imu")
        self.state = 0
        self.ax_slow = LowPass(time_factor=2.0) 
        self.ax_fast = LowPass(time_factor=0.2) 
        self.ay_slow = LowPass(time_factor=2.0) 
        self.ay_fast = LowPass(time_factor=0.2) 
        self.az_slow = LowPass(time_factor=2.0) 
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
                M = affine_matrix_from_points(v0, v1, shear=False, scale=True)
                R = M[:3,:3]
                print(R @ R.T)
                print("R:\n", R)
                # R should be orthogonal/normalized here
                # check if any row column doesn't have an element close to 1
                if np.max(np.abs(R[0])) < 0.9:
                    print("bad row 1")
                    self.state += 2
                elif np.max(np.abs(R[1])) < 0.9:
                    print("bad row 2")
                    self.state += 2
                elif np.max(np.abs(R[2])) < 0.9:
                    print("bad row 3")
                    self.state += 2
                elif np.max(np.abs(R[:,0])) < 0.9:
                    print("bad column 1")
                    self.state += 2
                elif np.max(np.abs(R[:,1])) < 0.9:
                    print("bad column 2")
                    self.state += 2
                elif np.max(np.abs(R[:,2])) < 0.9:
                    print("bad column 3")
                    self.state += 2
                else:
                    # nothing bad detected, save results and goto success state
                    scale, shear, angles, translate, perspective = decompose_matrix(M)
                    print("scale:", scale)
                    print("shear:", shear)
                    print("angles:", angles)
                    print("translate:", translate)
                    print("perspective:", perspective)

                    # recompose the original affine matrix with:
                    # translate @ rotate @ scale
                    
                    self.R = R
                    self.T = M[:,3] # translation vector
                    self.state += 1
        elif self.state == 7:
            # calibration complete, success, report!
            print("calibration succeeded")
            # consider current orientation matrix if it exists
            if self.config_imu_node and self.config_imu_node.hasChild("orientation"):
                current = []
                for i in range(9):
                    current.append(self.config_imu_node.getFloatEnum("orientation", i))
                current = np.array(current).reshape(3,3)
            else:
                current = np.eye(3)
            print("current:")
            print(current)
            print("R:")
            print(self.R)
            final = current @ self.R
            print("Final:")
            print(final)
            # as if this wasn't already fancy enough, get even fancier!
            errors = []
            for i, v in enumerate(self.meas):
                print("measure:", i, v)
                v1 = v @ self.R
                v0 = self.ref[i]
                err = np.linalg.norm(v0 - v1)
                errors.append(err)
            print("errors:", errors)
            mean = np.mean(errors)
            std = np.std(errors)
            print("calibration mean:", mean, " std:", std)
            self.state += 2
            calib_node = PropertyNode()
            calib_node.setLen("orientation", 9)
            for i in range(9):
                calib_node.setFloatEnum("orientation", i, final.flatten()[i])
            calib_node.setFloat("calibration_mean", mean)
            calib_node.setFloat("calibration_std", std)
            calib_node.setFloat("ax_bias", self.T[0])
            calib_node.setFloat("ay_bias", self.T[1])
            calib_node.setFloat("az_bias", self.T[2])
            logging_node = getNode("/config/logging", True)
            dir = logging_node.getString("flight_dir")
            props_json.save(os.path.join(dir, "imu_calib.json"), calib_node)
        elif self.state == 8:
            # calibration complete, but failed. :-(
            print("calibration failed")
            pass            

    def is_complete(self):
        return False

    def close(self):
        self.active = False
        return True

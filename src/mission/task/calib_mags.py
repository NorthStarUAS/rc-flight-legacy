import math
import numpy as np
import os
from scipy import linalg

from props import getNode, PropertyNode
import props_json

import comms.events
from mission.task.task import Task
from mission.task.lowpass import LowPass
import mission.task.transformations as tr

# state key:
#   0 = spin level
#   1 = spin nose down
#   2 = spin right wing up
#   3 = sanity check
#   4 = complete ok
#   5 = complete failed

def ellipsoid_fit(s):
    ''' Estimate ellipsoid parameters from a set of points.

        Parameters
        ----------
        s : array_like
          The samples (M,N) where M=3 (x,y,z) and N=number of samples.

        Returns
        -------
        M, n, d : array_like, array_like, float
          The ellipsoid parameters M, n, d.

        References
        ----------
        .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
           fitting," in Geometric Modeling and Processing, 2004.
           Proceedings, vol., no., pp.335-340, 2004
    '''

    # D (samples)
    D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                  2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                  2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

    # S, S_11, S_12, S_21, S_22 (eq. 11)
    S = np.dot(D, D.T)
    S_11 = S[:6,:6]
    S_12 = S[:6,6:]
    S_21 = S[6:,:6]
    S_22 = S[6:,6:]

    # C (Eq. 8, k=4)
    C = np.array([[-1,  1,  1,  0,  0,  0],
                  [ 1, -1,  1,  0,  0,  0],
                  [ 1,  1, -1,  0,  0,  0],
                  [ 0,  0,  0, -4,  0,  0],
                  [ 0,  0,  0,  0, -4,  0],
                  [ 0,  0,  0,  0,  0, -4]])

    # v_1 (eq. 15, solution)
    E = np.dot(linalg.inv(C),
               S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

    E_w, E_v = np.linalg.eig(E)

    v_1 = E_v[:, np.argmax(E_w)]
    if v_1[0] < 0: v_1 = -v_1

    # v_2 (eq. 13, solution)
    v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

    # quadric-form parameters
    M = np.array([[v_1[0], v_1[3], v_1[4]],
                  [v_1[3], v_1[1], v_1[5]],
                  [v_1[4], v_1[5], v_1[2]]])
    n = np.array([[v_2[0]],
                  [v_2[1]],
                  [v_2[2]]])
    d = v_2[3]

    return M, n, d
    
class CalibrateMagnetometer(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.imu_node = getNode("/sensors/imu", True)
        self.config_imu_node = getNode("/config/drivers/Aura4/imu")
        self.state = 0
        self.armed = False
        self.samples = []
        self.p_filt = LowPass(time_factor=0.1)
        self.q_filt = LowPass(time_factor=0.1)
        self.r_filt = LowPass(time_factor=0.1)
        self.ax_filt = LowPass(time_factor=0.2)
        self.ay_filt = LowPass(time_factor=0.2)
        self.az_filt = LowPass(time_factor=0.2)
        self.F   = F
        self.b   = np.zeros([3, 1])
        self.A_1 = np.eye(3)

    def activate(self):
        self.active = True
        self.armed = False
        self.samples = []
        self.rot = 0.0
        comms.events.log("calibrate magnetometer", "active")

    def detect_up(self):
        ax = self.ax_filt.filter_value
        ay = self.ay_filt.filter_value
        az = self.az_filt.filter_value
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
        p = self.imu_node.getFloat("p_rad_sec")
        q = self.imu_node.getFloat("q_rad_sec")
        r = self.imu_node.getFloat("r_rad_sec")
        ax = self.imu_node.getFloat("ax_mps_sec")
        ay = self.imu_node.getFloat("ay_mps_sec")
        az = self.imu_node.getFloat("az_mps_sec")
        hx_raw = self.imu_node.getFloat("hx_raw")
        hy_raw = self.imu_node.getFloat("hy_raw")
        hz_raw = self.imu_node.getFloat("hz_raw")
        self.p_filt.update(p, dt)
        self.q_filt.update(q, dt)
        self.r_filt.update(r, dt)
        self.ax_filt.update(ax, dt)
        self.ay_filt.update(ay, dt)
        self.az_filt.update(az, dt)

        spin = 0.2*math.pi
        #spin = 2.01*math.pi
        
        up_axis = self.detect_up()
        if up_axis == "none":
            self.armed = True
            self.rot = 0.0
            
        if self.state < 3:
            print("up axis:", up_axis, "armed:", self.armed, " rot: %0f" % self.rot)
              
        if self.state == 0:
            print("Spin 360 while holding level")
            if self.armed and self.detect_up() == "z-neg":
                self.rot += self.r_filt.filter_value * dt
                self.samples.append( [hx_raw, hy_raw, hz_raw] )
            if abs(self.rot) > spin:
                self.state += 1
                self.armed = False
                self.rot = 0.0
        elif self.state == 1:
            print("Spin 360 while holding nose down")
            if self.armed and self.detect_up() == "x-neg":
                self.rot += self.p_filt.filter_value * dt
                self.samples.append( [hx_raw, hy_raw, hz_raw] )
            if abs(self.rot) > spin:
                self.state += 1
                self.armed = False
                self.rot = 0.0
        elif self.state == 2:
            print("Spin 360 while holding right wing up")
            if self.armed and self.detect_up() == "y-neg":
                self.rot += self.q_filt.filter_value * dt
                self.samples.append( [hx_raw, hy_raw, hz_raw] )
            if abs(self.rot) > spin:
                self.state += 1
                self.armed = False
                self.rot = 0.0
        elif self.state == 3:
            # did we measure a bunch of samples?
            if len(self.samples) < 100:
                print("Somehow didn't get many samples. :-(")
                self.state += 2
            else:
                # ellipsoid fit with our sample data
                s = np.array(self.samples).T
                M, n, d = ellipsoid_fit(s)

                # calibration parameters.  Note: some implementations
                # of sqrtm return complex type, taking real
                M_1 = linalg.inv(M)
                self.b = -np.dot(M_1, n)
                self.A_1 = np.real(self.F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))

                print("b:\n", self.b)
                print("A_1:\n", self.A_1)
                
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
        elif self.state == 4:
            # calibration complete, success, report!
            print("calibration succeeded")
            print("strapdown calibration:")
            print(self.R)
            # as if this wasn't already fancy enough, get even fancier!
            errors = []
            for i, v in enumerate(self.meas):
                print("measure:", i, v)
                v1 =  self.accel_affine @ np.hstack((v, 1))
                v0 = self.ref[i]
                err = np.linalg.norm(v0 - v1[:3])
                errors.append(err)
            print("errors:", errors)
            mean = np.mean(errors)
            std = np.std(errors)
            print("calibration mean:", mean, " std:", std)
            self.state += 2
            node = PropertyNode()
            calib_node = node.getChild("calibration", True)
            calib_node.setLen("strapdown", 9)
            for i in range(9):
                calib_node.setFloatEnum("strapdown", i, self.R[:3,:3].flatten()[i])
            calib_node.setFloat("fit_mean", mean)
            calib_node.setFloat("fit_std", std)
            calib_node.setLen("accel_scale", 3)
            for i in range(3):
                calib_node.setFloatEnum("accel_scale", i, self.scale[i])
            calib_node.setLen("accel_translate", 3)
            for i in range(3):
                calib_node.setFloatEnum("accel_translate", i, self.translate[i])
            logging_node = getNode("/config/logging", True)
            dir = logging_node.getString("flight_dir")
            # props_json.save(os.path.join(dir, "imu_calib.json"), node)
        elif self.state == 5:
            # calibration complete, but failed. :-(
            print("calibration failed")
            pass            

    def is_complete(self):
        return False

    def close(self):
        self.active = False
        return True

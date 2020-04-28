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
        self.F   = 1.0          # fitted/output intensity
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
                
                # assemble the mag calibration matrix
                T = tr.translation_matrix(-self.b.flatten())
                A1_h = np.eye(4)
                A1_h[:3,:3] = self.A_1
                self.mag_affine = A1_h @ T # this is the correct order
                print("mag_affine:\n", self.mag_affine)
                scale, shear, angles, translate, perspective = tr.decompose_matrix(mag_affine)
                print("scale:", scale)
                print("shear:", shear)
                print("angles:", angles)
                print("translate:", translate)
                print("perspective:", perspective)
                # fixme what can we look at to sanity check?
                # nothing bad detected, goto success state
                self.state += 1
        elif self.state == 4:
            # calibration complete, success, report!
            print("calibration succeeded")
            print("magnetometer calibration:")
            # as if this wasn't already fancy enough, get even fancier!
            errors = []
            for i, v in enumerate(self.samples):
                print("sample:", i, v)
                v1 =  self.mag_affine @ np.hstack((v, 1))
                print(v, v1[:3])
            self.state += 2
            calib_node = self.config_imu_node.getChild("calibration", True)
            calib_node.setLen("mag_b", 3)
            for i in range(3):
                calib_node.setFloatEnum("mag_b", i, self.mag_b.flatten()[i])
            calib_node.setLen("mag_A_1", 9)
            for i in range(9):
                calib_node.setFloatEnum("mag_A_1", i, self.mag_A_1.flatten()[i])
            logging_node = getNode("/config/logging", True)
            dir = logging_node.getString("flight_dir")
            props_json.save(os.path.join(dir, "imu_calib.json"), calib_node)
        elif self.state == 5:
            # calibration complete, but failed. :-(
            print("calibration failed")
            pass            

    def is_complete(self):
        return False

    def close(self):
        self.active = False
        return True

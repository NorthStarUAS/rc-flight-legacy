import math
import numpy as np
import os
from scipy import linalg

from props import getNode, PropertyNode
import props_json

import comms.display
import comms.events
from mission.task.task import Task
from mission.task.lowpass import LowPass
import mission.task.transformations as tr

# state key:
#   0 = spend 5 seconds moving around at each major (6) orientation
#   1 = sanity check
#   2 = completed, success
#   3 = completed, failed

def ellipsoid_fit(s):
    # https://teslabs.com/articles/magnetometer-calibration/
    
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
        self.name = config_node.getString("name")
        self.imu_node = getNode("/sensors/imu", True)
        self.config_imu_node = getNode("/config/drivers/Aura4/imu", True)
        self.task_calib_node = getNode("/task/calibrate", True)
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
        self.axis_time = { "x-pos": 0, "x-neg": 0,
                           "y-pos": 0, "y-neg": 0,
                           "z-pos": 0, "z-neg": 0 }
        self.axis_hint = { "x-pos": "nose up", "x-neg": "nose down",
                           "y-pos": "right wing up", "y-neg": "right wing down",
                           "z-pos": "upside down", "z-neg": "top up" }
        comms.events.log("calibrate magnetometer", "active")
        self.min = [ 1000, 1000, 1000 ]    # debug
        self.max = [ -1000, -1000, -1000 ] # debug

    def detect_up(self):
        threshold = 7.5
        ax = self.ax_filt.filter_value
        ay = self.ay_filt.filter_value
        az = self.az_filt.filter_value
        if ax > threshold: return "x-pos"    # nose up
        elif ax < -threshold: return "x-neg" # nose down
        if ay > threshold: return "y-pos"    # right wing up
        elif ay < -threshold: return "y-neg" # right wing down
        if az > threshold: return "z-pos"    # up side down
        elif az < -threshold: return "z-neg" # right side up
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

        up_axis = self.detect_up()
        if up_axis == "none":
            self.armed = True
            self.rot = 0.0
            
        if self.state < 1:
            print("up axis:", up_axis, "armed:", self.armed, " rot: %0f" % self.rot)
        sample_time = 5
        
        if self.state == 0:
            if self.armed and up_axis != "none":
                if self.axis_time[up_axis] < sample_time:
                    self.samples.append( [hx_raw, hy_raw, hz_raw] )
                    if hx_raw < self.min[0]: self.min[0] = hx_raw
                    if hx_raw > self.max[0]: self.max[0] = hx_raw
                    if hy_raw < self.min[1]: self.min[1] = hy_raw
                    if hy_raw > self.max[1]: self.max[1] = hy_raw
                    if hz_raw < self.min[2]: self.min[2] = hz_raw
                    if hz_raw > self.max[2]: self.max[2] = hz_raw
                    print(self.min, self.max)
                self.axis_time[up_axis] += dt
            done = True
            for key in self.axis_time:
                if self.axis_time[key] < sample_time:
                    message = "need more: " + self.axis_hint[key]
                    self.task_calib_node.setString("mag_status", message)
                    comms.display.show(message)
                    done = False
                    break
            if done:
                self.state += 1
        elif self.state == 1:
            # did we measure a bunch of samples?
            if len(self.samples) < 100:
                print("Somehow didn't get many samples. :-(")
                self.state += 2
            else:
                # center, evecs, radii, v = ellipsoid_fit(s)
                # print("center:\n", center)
                # print("evecs:\n", evecs)
                # print("radii:\n", radii)
                # print("v:\n", v)
                
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
                scale, shear, angles, translate, perspective = tr.decompose_matrix(self.mag_affine)
                print("scale:", scale)
                print("shear:", shear)
                print("angles:", angles)
                print("translate:", translate)
                print("perspective:", perspective)
                # fixme what can we look at to sanity check?
                # nothing bad detected, goto success state
                self.state += 1
        elif self.state == 2:
            # calibration complete, success, save, report!
            print("calibration succeeded")
            print("magnetometer calibration:")
            # as if this wasn't already fancy enough, get even fancier!
            mapped = []
            for i, v in enumerate(self.samples):
                #print("sample:", i, v)
                v1 = self.mag_affine @ np.hstack((v, 1))
                v2 = self.A_1 @ (np.array(v) - self.b.flatten())
                #print(v, v1[:3], v2)
                mapped.append(v2)
            print("samples:")
            samples = np.array(self.samples)
            print("x range:", np.min(samples[:,0]), np.max(samples[:,0]))
            print("y range:", np.min(samples[:,1]), np.max(samples[:,1]))
            print("z range:", np.min(samples[:,2]), np.max(samples[:,2]))
            print("mapped:")
            mapped = np.array(mapped)
            print("x range:", np.min(mapped[:,0]), np.max(mapped[:,0]))
            print("y range:", np.min(mapped[:,1]), np.max(mapped[:,1]))
            print("z range:", np.min(mapped[:,2]), np.max(mapped[:,2]))
            self.state += 2
            calib_node = self.config_imu_node.getChild("calibration", True)
            calib_node.setLen("mag_affine", 16)
            for i in range(16):
                calib_node.setFloatEnum("mag_affine", i, self.mag_affine.flatten()[i])
            home = os.path.expanduser("~")
            props_json.save(os.path.join(home, "imu_calibration.json"), calib_node)
            message = "mag calibration succeeded"
            self.task_calib_node.setString("mag_status", message)
            comms.display.show(message)
            self.state += 2
        elif self.state == 3:
            # calibration complete, failed, sad face. :-(
            message = "mag calibration failed"
            self.task_calib_node.setString("mag_status", message)
            comms.display.show(message)
            self.state += 1

    def is_complete(self):
        if self.state >= 4:
            # free sample memory
            self.samples = []
            return True
        else:
            return False

    def close(self):
        self.active = False
        return True

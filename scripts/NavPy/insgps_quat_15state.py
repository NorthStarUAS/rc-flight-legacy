import numpy as np
import scipy.linalg as la
import navpy
import wgs84

class IMU():
    def __init__(self, time, valid, p, q, r, ax, ay, az, hx, hy, hz, temp):
        self.time = time
        self.valid = valid
        self.p = p
        self.q = q
        self.r = r
        self.ax = ax
        self.ay = ay
        self.az = az
        self.hx = hx
        self.hy = hy
        self.hz = hz
        self.temp = temp

class GPS():
    def __init__(self, time, valid, tow, lat, lon, alt, vn, ve, vd):
        self.time = time
        self.valid = valid
        self.tow = tow
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.vn = vn
        self.ve = ve
        self.vd = vd

class FILTER():
    def __init__(self, time, lat, lon, alt, vn, ve, vd, phi, the, psi):
        self.time = time
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.vn = vn
        self.ve = ve
        self.vd = vd
        self.phi = phi
        self.the = the
        self.psi = psi

class INSGPS():
    def __init__(self, valid, time, estPOS, estVEL, estATT, estAB, estGB,
                 P, stateInnov):
        self.valid = valid
        self.time = time
        self.estPOS = estPOS[:]
        self.estVEL = estVEL[:]
        self.estATT = estATT[:]
        self.estAB = estAB[:]
        self.estGB = estGB[:]
        self.P = P
        self.stateInnov = stateInnov[:]

class Filter():
    def __init__(self):
        # variable initializer
        self.very_first_time = True
        self.H = np.hstack( (np.eye(6), np.zeros((6,9))) )
        self.NAV_INIT = False
        self.IMU_CAL_INIT = False
        self.TU_COUNT = 0
        self.tcpu = -1.0
        self.last_tcpu = -1.0
        self.tow = -1.0
        self.last_tow = -1.0

        # Initialize process noise (Rw, tau_a, tau_g) with default values
        self.init_process_noise()

        # initialize measurement noise (R) with default values
        self.init_measurement_noise()

        # Initialize covariance (P)
        self.init_covariance()

        # Initialize filter outputs
        self.estPOS = np.nan*np.ones(3)
        self.estVEL = np.nan*np.ones(3)
        self.estATT = np.nan*np.ones(4)
        self.estAB = np.nan*np.ones(3)
        self.estGB = np.nan*np.ones(3)
        self.last_estPOS = self.estPOS[:]
        self.last_estVEL = self.estVEL[:]
        self.last_estATT = self.estATT[:]
        self.last_estAB = self.estAB[:]
        self.last_estGB = self.estGB[:]

        # placeholders
        self.stateInnov = np.nan*np.ones(6)

    def init_process_noise(self,
                           sig_w_ax=0.05, sig_w_ay=0.05, sig_w_az=0.05,
                           sig_w_gx_deg=0.1, sig_w_gy_deg=0.1, sig_w_gz_deg=0.1,
                           sig_a_d_g=5e-3, sig_g_d_deg=0.05,
                           tau_a=100.0, tau_g=50.0):
        # initialize process noise
        # white noise accelerometers: sig_w_ax, sig_w_ay, sig_w_az
        # white noise gyros: sig_w_gx, sig_w_gy, sig_w_gz
        # time-correlated noise accelerometers: sig_a_d
        # time-correlated noise gyros: sig_g_d
        # and then the tau's
        self.tau_a = tau_a
        self.tau_g = tau_g

        sig_w_gx = np.deg2rad(sig_w_gx_deg)
        sig_w_gy = np.deg2rad(sig_w_gy_deg)
        sig_w_gz = np.deg2rad(sig_w_gz_deg)
        sig_a_d = sig_a_d_g*9.81
        sig_g_d = np.deg2rad(sig_g_d_deg)

        self.Rw = np.diag([sig_w_ax**2, sig_w_ay**2, sig_w_az**2,
                           sig_w_gx**2, sig_w_gy**2, sig_w_gz**2,
                           2*sig_a_d**2/self.tau_a, 2*sig_a_d**2/self.tau_a,
                           2*sig_a_d**2/self.tau_a, 2*sig_g_d**2/self.tau_g,
                           2*sig_g_d**2/self.tau_g, 2*sig_g_d**2/self.tau_g])

    def init_measurement_noise(self,
                               sig_gps_p_ne=3.0, sig_gps_p_d=5.0,
                               sig_gps_v=0.5):
        # measurement noise:
        # Values are inflated because GPS antennas are located off CG
        # and not compensated
        # horizontal: sig_gps_p_ne
        # vertical: sig_gps_p_d
        # velocity: sig_gps_v
        self.R = np.diag([sig_gps_p_ne**2, sig_gps_p_ne**2, sig_gps_p_d**2,
                          sig_gps_v**2, sig_gps_v**2, sig_gps_v**2])


    def init_covariance(self):
        # initialize the covariance matrix (P)
        self.P = np.diag([10, 10, 10,\
                          1, 1, 1,\
                          np.deg2rad(10), np.deg2rad(10), np.deg2rad(90),\
                          1e-2*9.81, 1e-2*9.81, 1e-2*9.81,\
                          np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5)])
        self.P = self.P * self.P
        #Pp[i,:] = np.diag(self.P[0:3,0:3])
        #Pvel[i,:] = np.diag(self.P[3:6,3:6])
        #Patt[i,:] = np.diag(self.P[6:9,6:9])
        #Pab[i,:] = np.diag(self.P[9:12,9:12])
        #Pgb[i,:] = np.diag(self.P[12:15,12:15])

    def update(self, imu, gps, verbose=True):
        # Check for validity of GNSS at this time
        self.tcpu = imu.time
        self.tow = gps.tow

        # Test 1: navValid flag and the data is indeed new (new Time of Week)
        # Execute Measurement Update if this is true and Test 2 passes
        NEW_GNSS_FLAG = ((gps.valid==0) & (abs(self.tow-self.last_tow) > 1e-3))

        # Test 2: Check if the delta time of the Time of Week and the
        # CPU time are consistent
        # If this fails, re-initialize the filter. There must have been a glitch
        if NEW_GNSS_FLAG:
            #print self.tcpu, self.last_tcpu, self.tcpu-self.last_tcpu
            #print self.tow, self.last_tow, self.tow-self.last_tow
            if abs((self.tcpu-self.last_tcpu) - (self.tow-self.last_tow)) > 0.5:
                self.TU_COUNT = 0
                self.NAV_INIT = False
                if verbose:
                    print("Time Sync Error -- Request reinitialization")
            # Record the tow and tcpu at this new update
            self.last_tow = self.tow
            self.last_tcpu = self.tcpu

        # Different subroutine executed in the filter
        if not self.NAV_INIT:
            if not self.IMU_CAL_INIT:
                # SUBROUTINE: IMU CALIBRATION,
                # This only happens first time round on the ground. Inflight
                # reinitialization is not the same.
                self.estAB[:] = [0,0,0]
                if self.very_first_time:
                    self.very_first_time = False
                    self.estGB[:] = [imu.p, imu.q, imu.r]
                    self.phi = 0
                    self.theta = 0
                else:
                    self.estGB[:] = self.last_estGB[:]*self.TU_COUNT \
                                    + [imu.p, imu.q, imu.r]
                    # Simple AHRS values
                    self.phi = self.phi*self.TU_COUNT \
                               + np.arctan2(-imu.ay, -imu.az)
                    self.theta = self.theta*self.TU_COUNT \
                                 + np.arctan2(imu.ax, np.sqrt(imu.ay**2+imu.az**2))

                self.phi /= (self.TU_COUNT + 1)
                self.theta /= (self.TU_COUNT + 1)
                self.estGB[:] /= (self.TU_COUNT + 1)
                self.estATT[0], self.estATT[1:] = navpy.angle2quat(0, self.theta, self.phi)

                """
                print("t = %7.3f, Gyro Bias Value: [%6.2f, %6.2f, %6.2f] deg/sec" %\
                      (imu.time, np.rad2deg(self.estGB[0]), np.rad2deg(self.estGB[1]), np.rad2deg(self.estGB[2]) ))
                print("t = %7.3f, phi = %6.2f, theta = %6.2f" % (imu.time,np.rad2deg(self.phi),np.rad2deg(self.theta)) )
                """

                self.TU_COUNT += 1
                if self.TU_COUNT >= 35:
                    self.TU_COUNT = 0
                    self.IMU_CAL_INIT = True
                    if verbose:
                        print("t = %7.3f, IMU Calibrated!" % (imu.time))
                    del(self.phi)
                    del(self.theta)
            else:
                if not NEW_GNSS_FLAG:
                    # SUBROUTINE 1: BACKUP NAVIGATION or IN-FLIGHT INITIALIZATION

                    # >>>> AHRS CODE GOES HERE
                    self.estATT[:] = self.last_estATT[:]  # This should be some backup nav mode
                    # <<<<

                    # When still there is no GNSS signal, continue propagating bias
                    self.estAB[:] = self.last_estAB[:]
                    self.estGB[:] = self.last_estGB[:]
                else:
                    # When there is GNSS fix available, initialize all the states
                    # and renew covariance
                    self.estPOS[:] = [gps.lat, gps.lon, gps.alt]
                    self.estVEL[:] = [gps.vn, gps.ve, gps.vd]
                    self.estATT[:] = self.last_estATT[:]

                    #self.estATT[0], self.estATT[1:] = navpy.angle2quat(flight_data.psi[i],flight_data.theta[i],flight_data.phi[i])

                    self.estAB[:] = self.last_estAB[:]
                    self.estGB[:] = self.last_estGB[:]

                    self.init_covariance()

                    #idx_init.append(i)
                    self.NAV_INIT = True
                    if verbose:
                        print("t = %7.3f, Filter (Re-)initialized" % (imu.time) )

        elif self.NAV_INIT:
            # SUBROUTINE 2: MAIN FILTER ALGORITHM, INS + GNSS
            # ==== Time-Update ====
            dt = imu.time - self.last_imu.time
            q0, qvec = self.last_estATT[0], self.last_estATT[1:4]

            C_B2N = navpy.quat2dcm(q0,qvec).T

            # 0. Data Acquisition
            f_b=np.array([0.5*(self.last_imu.ax+imu.ax)-self.last_estAB[0],\
                          0.5*(self.last_imu.ay+imu.ay)-self.last_estAB[1],\
                          0.5*(self.last_imu.az+imu.az)-self.last_estAB[2]])

            om_ib=np.array([0.5*(self.last_imu.p+imu.p)-self.last_estGB[0],\
                            0.5*(self.last_imu.q+imu.q)-self.last_estGB[1],\
                            0.5*(self.last_imu.r+imu.r)-self.last_estGB[2]])

            # 1. Attitude Update
            # --> Need to compensate for navrate and earthrate
            dqvec = 0.5*om_ib*dt
            self.estATT[0], self.estATT[1:4] = navpy.qmult(q0,qvec,1.0,dqvec)

            self.estATT[0] /= np.sqrt(self.estATT[0]**2 \
                                      + la.norm(self.estATT[1:4])**2)
            self.estATT[1:4] /= np.sqrt(self.estATT[0]**2 \
                                        + la.norm(self.estATT[1:4])**2)

            # 2. Velocity Update
            # --> Need to compensate for coriolis effect
            g = np.array([0,0,9.81])
            f0, fvec = navpy.qmult(q0,qvec,0,f_b)
            f_n0,f_nvec = navpy.qmult(f0,fvec,q0,-qvec)
            self.estVEL[:] = self.last_estVEL[:] + (f_nvec+g)*dt

            # 3. Position Update
            dPOS = navpy.llarate(self.last_estVEL[0], self.last_estVEL[1],
                                 self.last_estVEL[2], self.last_estPOS[0],
                                 self.last_estPOS[2])
            dPOS *= dt
            self.estPOS[:] = self.last_estPOS[:] + dPOS

            # 4. Biases are constant
            self.estAB[:] = self.last_estAB[:]
            self.estGB[:] = self.last_estGB[:]

            # 5. Jacobian
            pos2pos = np.zeros((3,3))
            pos2gs = np.eye(3)
            pos2att = np.zeros((3,3))
            pos2acc = np.zeros((3,3))
            pos2gyr = np.zeros((3,3))

            gs2pos = np.zeros((3,3))
            gs2pos[2,2] = -2*9.81/wgs84.R0
            gs2gs = np.zeros((3,3))
            gs2att = -2*C_B2N.dot(navpy.skew(f_b))
            gs2acc = -C_B2N
            gs2gyr = np.zeros((3,3))

            att2pos = np.zeros((3,3))
            att2gs = np.zeros((3,3))
            att2att = -navpy.skew(om_ib)
            att2acc = np.zeros((3,3))
            att2gyr = -0.5*np.eye(3)

            F = np.zeros((15,15))
            F[0:3,0:3] = pos2pos
            F[0:3,3:6] = pos2gs
            F[0:3,6:9] = pos2att
            F[0:3,9:12] = pos2acc
            F[0:3,12:15] = pos2gyr

            F[3:6,0:3] = gs2pos
            F[3:6,3:6] = gs2gs
            F[3:6,6:9] = gs2att
            F[3:6,9:12] = gs2acc
            F[3:6,12:15] = gs2gyr

            F[6:9,0:3] = att2pos
            F[6:9,3:6] = att2gs
            F[6:9,6:9] = att2att
            F[6:9,9:12] = att2acc
            F[6:9,12:15] = att2gyr

            F[9:12,9:12] = -1.0/self.tau_a*np.eye(3)
            F[12:15,12:15] = -1.0/self.tau_g*np.eye(3)

            PHI = np.eye(15) + F*dt

            # 6. Process Noise
            G = np.zeros((15,12))
            G[3:6,0:3] = -C_B2N
            G[6:9,3:6] = att2gyr
            G[9:12,6:9] = np.eye(3)
            G[12:15,9:12] = np.eye(3)

            Q = G.dot(self.Rw.dot(G.T))*dt

            # 7. Covariance Update
            self.P = PHI.dot(self.P.dot(PHI.T)) + Q

            self.TU_COUNT += 1

            if self.TU_COUNT >= 500:
                # Request reinitialization after 12 seconds of no GNSS updates
                self.TU_COUNT = 0
                self.NAV_INIT = False

            if NEW_GNSS_FLAG:
                # ==== Measurement-Update ====
                # 0. Get measurement and make innovations
                ecef_ref = navpy.lla2ecef(self.estPOS[0], self.estPOS[1], 0)
                ins_ecef = navpy.lla2ecef(self.estPOS[0], self.estPOS[1],
                                          self.estPOS[2])
                gnss_ecef = navpy.lla2ecef(gps.lat, gps.lon, gps.alt)

                ins_ned = navpy.ecef2ned(ins_ecef-ecef_ref, self.estPOS[0],
                                         self.estPOS[1], self.estPOS[2])
                gnss_ned = navpy.ecef2ned(gnss_ecef-ecef_ref, self.estPOS[0],
                                          self.estPOS[1], self.estPOS[2])

                dpos = gnss_ned - ins_ned

                gnss_vel = np.array([gps.vn, gps.ve, gps.vd])
                dvel = gnss_vel - self.estVEL[:]

                dy = np.hstack((dpos, dvel))
                self.stateInnov[:] = dy

                # 1. Kalman Gain
                K = self.P.dot(self.H.T)
                K = K.dot( la.inv(self.H.dot(self.P.dot(self.H.T)) + self.R) )

                # 2. Covariance Update
                ImKH = np.eye(15) - K.dot(self.H)
                KRKt = K.dot(self.R.dot(K.T))
                self.P = ImKH.dot(self.P.dot(ImKH.T)) + KRKt

                # 3. State Update
                dx = K.dot(dy)

                Rew, Rns = navpy.earthrad(self.estPOS[0])

                self.estPOS[2] -= dx[2]
                self.estPOS[0] += np.rad2deg(dx[0]/(Rew + self.estPOS[2]))
                self.estPOS[1] += np.rad2deg(dx[1]/(Rns + self.estPOS[2])/np.cos(np.deg2rad(self.estPOS[0])))

                self.estVEL[:] += dx[3:6]

                self.estATT[0], self.estATT[1:4] = navpy.qmult(self.estATT[0], self.estATT[1:4], 1, dx[6:9])

                self.estATT[0] /= np.sqrt(self.estATT[0]**2 \
                                          + la.norm(self.estATT[1:4])**2)
                self.estATT[1:4] /= np.sqrt(self.estATT[0]**2 \
                                            + la.norm(self.estATT[1:4])**2)

                self.estAB[:] += dx[9:12]
                self.estGB[:] += dx[12:15]

                if verbose:
                    print("t = %7.3f, GNSS Update, self.TU_COUNT = %d" %\
                          (gps.time, self.TU_COUNT) )

                self.TU_COUNT = 0

                self.last_estPOS[:] = self.estPOS[:]
                self.last_estVEL[:] = self.estVEL[:]
                self.last_estATT[:] = self.estATT[:]
                self.last_estAB[:] = self.estAB[:]
                self.last_estGB[:] = self.estGB[:]
        else:
            # SUBROUTINE 3: e.g. BACKUP NAVIGATION MODE
            pass

        self.last_imu = imu
        self.last_gps = gps

        result = INSGPS( self.NAV_INIT, imu.time, self.estPOS, self.estVEL,
                         self.estATT, self.estAB, self.estGB,
                         self.P, self.stateInnov )
        return result

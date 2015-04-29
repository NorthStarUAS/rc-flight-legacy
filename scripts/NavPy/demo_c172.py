from loading_mat_data import load_struct
import numpy as np
import matplotlib.pyplot as plt
import insgps_quat_15state
import navpy

np.set_printoptions(precision=5,suppress=True)
plt.close()

# >>> UMN UAV Flight Data
#path = '/Users/adhika/Dropbox/python_code/SysID/data/'
#fname = 'ibis_flight05_doublets_2013_07_07_cal.mat'
#fname = 'thor_flight91_doublet_claw_2013_07_07.mat'
# <<<

# >>> Flight Gear Simulation Data
path = ''
fname = 'data/C172_10042012mod.mat'
# <<<

flight_data, flight_info = load_struct(path+fname)

# ============================ NOISE CONFIGURATION =============================
# --- Process Noise
# White Noise Part
#sig_w_ax = 0.3
#sig_w_ay = 0.3
#sig_w_az = 0.3
#sig_w_gx = np.deg2rad(0.3)
#sig_w_gy = np.deg2rad(0.3)
#sig_w_gz = np.deg2rad(0.3)
# Time-correlated Part
#sig_a_d = 5e-3*9.81
#tau_a =   100.0
#sig_g_d = np.deg2rad(0.05)
#tau_g = 50.0

#Rw = np.diag([sig_w_ax**2, sig_w_ay**2, sig_w_az**2,
#              sig_w_gx**2, sig_w_gy**2, sig_w_gz**2,
#              2*sig_a_d**2/tau_a, 2*sig_a_d**2/tau_a, 2*sig_a_d**2/tau_a,
#              2*sig_g_d**2/tau_g, 2*sig_g_d**2/tau_g, 2*sig_g_d**2/tau_g])

# --- Measurement Noise
#sig_gps_p_ne = 3;
#sig_gps_p_d = 5;
#sig_gps_v = 0.5;  # Inflated, GPS antennas are located off CG and not compensated

#R = np.diag([sig_gps_p_ne**2, sig_gps_p_ne**2, sig_gps_p_d**2,
#             sig_gps_v**2, sig_gps_v**2, sig_gps_v**2])

# ===========================   PLACEHOLDERS ===============================
drl = len(flight_data.time)

estPOS = np.nan*np.ones((drl,3))
estVEL = np.nan*np.ones((drl,3))
estATT = np.nan*np.ones((drl,4))
estAB = np.nan*np.ones((drl,3))
estGB = np.nan*np.ones((drl,3))

Pp = np.nan*np.ones((drl,3))
Pvel = np.nan*np.ones((drl,3))
Patt = np.nan*np.ones((drl,3))
Pab = np.nan*np.ones((drl,3))
Pgb = np.nan*np.ones((drl,3))

stateInnov = np.nan*np.ones((drl,6))

# ============================ VARIABLE INITIALIZER ============================
# moved to init(): H = np.hstack( (np.eye(6), np.zeros((6,9))) )
# moved to init(): NAV_INIT = False
# moved to init(): IMU_CAL_INIT = False
# moved to init(): TU_COUNT = 0

idx_init = []

# moved to init(): tcpu = -1.0; old_tcpu = -1.0
# moved to init(): tow = -1.0; old_tow = -1.0

# =============================== MAIN LOOP ====================================

filter = insgps_quat_15state.Filter()
nav_init = False
for i in range(0,drl):
    # prepare the sensor data
    imu = insgps_quat_15state.IMU( flight_data.time[i], flight_data.navValid[i],
                     flight_data.p[i], flight_data.q[i], flight_data.r[i],
                     flight_data.ax[i], flight_data.ay[i], flight_data.az[i] )
    gps = insgps_quat_15state.GPS( flight_data.time[i], flight_data.navValid[i],
                     flight_data.GPS_TOW[i],
                     flight_data.lat[i], flight_data.lon[i],
                     flight_data.alt[i],
                     flight_data.vn[i], flight_data.ve[i], flight_data.vd[i] )

    # update the filter
    est = filter.update(imu, gps)

    # save the results for plotting
    if not nav_init and est.valid:
        nav_init = True
        idx_init.append(i)
    elif not est.valid:
        nav_init = False
    estPOS[i,:] = est.estPOS[:]
    estVEL[i,:] = est.estVEL[:]
    estATT[i,:] = est.estATT[:]
    estAB[i,:] = est.estAB[:]
    estGB[i,:] = est.estGB[:]
    Pp[i,:] = np.diag(est.P[0:3,0:3])
    Pvel[i,:] = np.diag(est.P[3:6,3:6])
    Patt[i,:] = np.diag(est.P[6:9,6:9])
    Pab[i,:] = np.diag(est.P[9:12,9:12])
    Pgb[i,:] = np.diag(est.P[12:15,12:15])
    stateInnov[i,:] = est.stateInnov[:]

psi, theta, phi = navpy.quat2angle(estATT[:,0],estATT[:,1:4],output_unit='deg')

# Calculate Attitude Error
delta_att = np.nan*np.zeros((drl,3))
for i in range(0,drl):
    C1 = navpy.angle2dcm(flight_data.psi[i],flight_data.theta[i],flight_data.phi[i]).T
    C2 = navpy.angle2dcm(psi[i],theta[i],phi[i],input_unit='deg').T
    dC = C2.dot(C1.T)-np.eye(3)
    # dC contains delta angle. To match delta quaternion, divide by 2.
    delta_att[i,:] = [-dC[1,2]/2.0, dC[0,2]/2.0, -dC[0,1]/2.0]

lat_ref = estPOS[idx_init[0],0]
lon_ref = estPOS[idx_init[0],1]
alt_ref = estPOS[idx_init[0],2]

ecef_ref = navpy.lla2ecef(lat_ref,lon_ref,alt_ref)

ecef = navpy.lla2ecef(estPOS[:,0],estPOS[:,1],estPOS[:,2])
filter_ned = navpy.ecef2ned(ecef-ecef_ref,lat_ref,lon_ref,alt_ref)

ecef = navpy.lla2ecef(flight_data.lat,flight_data.lon,flight_data.alt)
gnss_ned = navpy.ecef2ned(ecef-ecef_ref,lat_ref,lon_ref,alt_ref)
gnss_ned[0:idx_init[0],:] = np.nan

ecef = navpy.lla2ecef(flight_data.navlat,flight_data.navlon,flight_data.navalt,latlon_unit='rad')
ref_ned = navpy.ecef2ned(ecef-ecef_ref,lat_ref,lon_ref,alt_ref)
ref_ned[0:idx_init[0],:] = np.nan

# ============================= INS PLOTS ======================================
nsig = 3
istart = idx_init[0]
istop = drl

pos_fig, pos_ax = plt.subplots(1)
pos_ax.plot(gnss_ned[:,1],gnss_ned[:,0],'*',label='GNSS')
pos_ax.plot(ref_ned[:,1],ref_ned[:,0],label='Ref')
pos_ax.plot(filter_ned[:,1],filter_ned[:,0],label='Filter')
pos_ax.set_title('Location')
pos_ax.set_ylabel('North (m)')
pos_ax.set_xlabel('East (m)')
pos_ax.legend(loc='best')
pos_ax.set_aspect('equal')

vel_fig, vel_ax = plt.subplots(3,2, sharex=True)
vel_ax[0,0].plot(flight_data.time[istart:istop],flight_data.navvn[istart:istop],label='True')
vel_ax[0,0].plot(flight_data.time[istart:istop],estVEL[istart:istop,0],'r',label='Filter')
vel_ax[0,0].set_ylabel('$V_N$ (m/s)')
vel_ax[0,0].legend()

vel_ax[1,0].plot(flight_data.time[istart:istop],flight_data.navve[istart:istop],label='True')
vel_ax[1,0].plot(flight_data.time[istart:istop],estVEL[istart:istop,1],'r',label='Filter')
vel_ax[1,0].set_ylabel('$V_E$ (m/s)')

vel_ax[2,0].plot(flight_data.time[istart:istop],flight_data.navvd[istart:istop],label='True')
vel_ax[2,0].plot(flight_data.time[istart:istop],estVEL[istart:istop,2],'r',label='Filter')
vel_ax[2,0].set_ylabel('$V_D$ (m/s)')

vel_ax[0,1].plot(flight_data.time[istart:istop],flight_data.navvn[istart:istop]-estVEL[istart:istop,0],'r')
vel_ax[0,1].plot(flight_data.time[istart:istop],nsig*np.sqrt(Pvel[istart:istop,0]),'k')
vel_ax[0,1].plot(flight_data.time[istart:istop],-nsig*np.sqrt(Pvel[istart:istop,0]),'k')

vel_ax[1,1].plot(flight_data.time[istart:istop],flight_data.navve[istart:istop]-estVEL[istart:istop,1],'r')
vel_ax[1,1].plot(flight_data.time[istart:istop],nsig*np.sqrt(Pvel[istart:istop,1]),'k')
vel_ax[1,1].plot(flight_data.time[istart:istop],-nsig*np.sqrt(Pvel[istart:istop,1]),'k')

vel_ax[2,1].plot(flight_data.time[istart:istop],flight_data.navvd[istart:istop]-estVEL[istart:istop,2],'r')
vel_ax[2,1].plot(flight_data.time[istart:istop],nsig*np.sqrt(Pvel[istart:istop,2]),'k')
vel_ax[2,1].plot(flight_data.time[istart:istop],-nsig*np.sqrt(Pvel[istart:istop,2]),'k')

vel_ax[0,0].set_title('Velocity (m/s)')
vel_ax[0,1].set_title('Velocity Error (m/s)')
vel_ax[2,0].set_xlabel('Time (sec)')
vel_ax[2,1].set_xlabel('Time (sec)')

att_fig, att_ax = plt.subplots(3,2, sharex=True)
att_ax[0,0].plot(flight_data.time[istart:istop],np.rad2deg(flight_data.phi[istart:istop]),label='True')
att_ax[0,0].plot(flight_data.time[istart:istop],phi[istart:istop],'r',label='Filter')
att_ax[0,0].set_ylabel(r'$phi$ (deg)')
att_ax[0,0].legend()

att_ax[1,0].plot(flight_data.time[istart:istop],np.rad2deg(flight_data.theta[istart:istop]),label='True')
att_ax[1,0].plot(flight_data.time[istart:istop],theta[istart:istop],'r',label='Filter')
att_ax[1,0].set_ylabel(r'$theta$ (deg)')

att_ax[2,0].plot(flight_data.time[istart:istop],np.rad2deg(flight_data.psi[istart:istop]),label='True')
att_ax[2,0].plot(flight_data.time[istart:istop],psi[istart:istop],'r',label='Filter')
att_ax[2,0].set_ylabel(r'$psi$ (deg)')

att_ax[0,1].plot(flight_data.time[istart:istop],np.rad2deg(delta_att[istart:istop,0]),'r')
att_ax[0,1].plot(flight_data.time[istart:istop],nsig*np.rad2deg(np.sqrt(Patt[istart:istop,0])),'k')
att_ax[0,1].plot(flight_data.time[istart:istop],-nsig*np.rad2deg(np.sqrt(Patt[istart:istop,0])),'k')

att_ax[1,1].plot(flight_data.time[istart:istop],np.rad2deg(delta_att[istart:istop,1]),'r')
att_ax[1,1].plot(flight_data.time[istart:istop],nsig*np.rad2deg(np.sqrt(Patt[istart:istop,1])),'k')
att_ax[1,1].plot(flight_data.time[istart:istop],-nsig*np.rad2deg(np.sqrt(Patt[istart:istop,1])),'k')

att_ax[2,1].plot(flight_data.time[istart:istop],np.rad2deg(delta_att[istart:istop,2]),'r')
att_ax[2,1].plot(flight_data.time[istart:istop],nsig*np.rad2deg(np.sqrt(Patt[istart:istop,2])),'k')
att_ax[2,1].plot(flight_data.time[istart:istop],-nsig*np.rad2deg(np.sqrt(Patt[istart:istop,2])),'k')

att_ax[0,0].set_title('Euler Angle (deg)')
att_ax[0,1].set_title('Attitude Error (deg)')
att_ax[2,0].set_xlabel('Time (sec)')
att_ax[2,1].set_xlabel('Time (sec)')

ab_fig, ab_ax = plt.subplots(3, sharex=True)
try:
    ab_ax[0].plot(flight_data.time[istart:istop],flight_data.ax_bias[istart:istop],label='True')
except AttributeError:
    pass
ab_ax[0].plot(flight_data.time[istart:istop],estAB[istart:istop,0],label='Filter')
ab_ax[0].set_ylabel('$b_{ax}$ (m/s$^2$)')
ab_ax[0].set_title('Accelerometer Bias')

try:
    ab_ax[1].plot(flight_data.time[istart:istop],flight_data.ay_bias[istart:istop],label='True')
except AttributeError:
    pass
ab_ax[1].plot(flight_data.time[istart:istop],estAB[istart:istop,1],label='Filter')
ab_ax[1].set_ylabel('$b_{ay}$ (m/s$^2$)')
ab_ax[1].legend(loc='best')

try:
    ab_ax[2].plot(flight_data.time[istart:istop],flight_data.az_bias[istart:istop],label='True')
except AttributeError:
    pass
ab_ax[2].plot(flight_data.time[istart:istop],estAB[istart:istop,2],label='Filter')
ab_ax[2].set_ylabel('$b_{az}$ (m/s$^2$)')
ab_ax[2].set_xlabel('Time (sec)')

gb_fig, gb_ax = plt.subplots(3, sharex=True)
try:
    gb_ax[0].plot(flight_data.time[istart:istop],np.rad2deg(flight_data.p_bias[istart:istop]),label='True')
except AttributeError:
    pass
gb_ax[0].plot(flight_data.time[istart:istop],np.rad2deg(estGB[istart:istop,0]),label='Filter')
gb_ax[0].set_ylabel('$b_{gx}$ (deg/s)')
gb_ax[0].set_title('Gyro Bias')

try:
    gb_ax[1].plot(flight_data.time[istart:istop],np.rad2deg(flight_data.q_bias[istart:istop]),label='True')
except AttributeError:
    pass
gb_ax[1].plot(flight_data.time[istart:istop],np.rad2deg(estGB[istart:istop,1]),label='Filter')
gb_ax[1].set_ylabel('$b_{gy}$ (deg/s)')
gb_ax[1].legend(loc='best')

try:
    gb_ax[2].plot(flight_data.time[istart:istop],np.rad2deg(flight_data.r_bias[istart:istop]),label='True')
except AttributeError:
    pass
gb_ax[2].plot(flight_data.time[istart:istop],np.rad2deg(estGB[istart:istop,2]),label='Filter')
gb_ax[2].set_ylabel('$b_{gz}$ (deg/s)')
gb_ax[2].set_xlabel('Time (sec)')

plt.show()



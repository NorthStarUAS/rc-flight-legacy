import os.path
import sys
import fileinput

import numpy as np
import matplotlib.pyplot as plt
import insgps_quat_15state
import navpy

np.set_printoptions(precision=5,suppress=True)
plt.close()

def usage():
    print "Usage: " + sys.argv[0] + " <flight_dir>"

if len(sys.argv) != 2:
    usage()
    sys.exit()

flight_path = sys.argv[1]

# load imu/gps data files
imu_file = flight_path + "/imu.txt"
gps_file = flight_path + "/gps.txt"

imu_data = []
fimu = fileinput.input(imu_file)
for line in fimu:
    time, p, q, r, ax, ay, az, hx, hy, hz, temp, status = line.split()
    imu = insgps_quat_15state.IMU( float(time), int(status),
                                   float(p), float(q), float(r),
                                   float(ax), float(ay), float(az),
                                   float(hx), float(hy), float(hz),
                                   float(temp) )
    imu_data.append( imu )
if len(imu_data) == 0:
    print "No imu records loaded, cannot continue..."
    sys.exit()

gps_data = []
fgps = fileinput.input(gps_file)
for line in fgps:
    # note the avior logs unix time of the gps record, not tow, but
    # for the pruposes of the insgns algorithm, it's only important to
    # have a properly incrementing clock, it doens't really matter
    # what the zero reference point of time is.
    time, lat, lon, alt, vn, ve, vd, unixsec, sats, status = line.split()
    gps = insgps_quat_15state.GPS( float(time), int(status), float(unixsec),
                                   float(lat), float(lon), float(alt),
                                   float(vn), float(ve), float(vd))
    gps_data.append(gps)
if len(gps_data) == 0:
    print "No gps records loaded, cannot continue..."
    sys.exit()

# =========================== Results ===============================
drl = len(imu_data)

estPOS = np.nan*np.ones((drl,3))
estVEL = np.nan*np.ones((drl,3))
estATT = np.nan*np.ones((drl,4))
estAB = np.nan*np.ones((drl,4))
estGB = np.nan*np.ones((drl,4))

Pp = np.nan*np.ones((drl,3))
Pvel = np.nan*np.ones((drl,3))
Patt = np.nan*np.ones((drl,3))
Pab = np.nan*np.ones((drl,3))
Pgb = np.nan*np.ones((drl,3))

stateInnov = np.nan*np.ones((drl,6))

plot_time = [0.0] * drl

# ============================ VARIABLE INITIALIZER ============================
idx_init = []

# =============================== MAIN LOOP ====================================

filter = insgps_quat_15state.Filter()

gps_index = 1
nav_init = False
for i, imu in enumerate(imu_data):
    # walk the gps counter forward as needed
    if imu.time >= gps_data[gps_index].time:
        gps_index += 1
    if gps_index >= len(gps_data):
        # no more gps data, stick on the last record
        gps_index = len(gps_data)-1
    gps = gps_data[gps_index-1]
    # print "t(imu) = " + str(imu.time) + " t(gps) = " + str(gps.time)

    # update the filter
    plot_time[i] = imu.time
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
    estAB[i,0:3] = est.estAB[:]
    estAB[i,3] = imu.temp
    estGB[i,0:3] = est.estGB[:]
    estGB[i,3] = imu.temp
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
    # when processing real data, we have no truth reference
    #C1 = navpy.angle2dcm(flight_data.psi[i],flight_data.theta[i],flight_data.phi[i]).T
    C1 = navpy.angle2dcm(psi[i],theta[i],phi[i],input_unit='deg').T
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

# when processing real data, we have no truth reference
#ecef = navpy.lla2ecef(flight_data.lat,flight_data.lon,flight_data.alt)
#ecef = navpy.lla2ecef(estPOS[:,0],estPOS[:,1],estPOS[:,2])
#gnss_ned = navpy.ecef2ned(ecef-ecef_ref,lat_ref,lon_ref,alt_ref)
#gnss_ned[0:idx_init[0],:] = np.nan

rawgps = np.nan*np.ones((len(gps_data),3))
for i, gps in enumerate(gps_data):
    rawgps[i,:] = [ gps.lat, gps.lon, gps.alt ]
ecef = navpy.lla2ecef(rawgps[:,0],rawgps[:,1],rawgps[:,2],latlon_unit='deg')
ref_ned = navpy.ecef2ned(ecef-ecef_ref,lat_ref,lon_ref,alt_ref)
ref_ned[0:idx_init[0],:] = np.nan

# ============================= INS PLOTS ======================================
nsig = 3
istart = idx_init[0]
istop = drl

pos_fig, pos_ax = plt.subplots(1)
#pos_ax.plot(gnss_ned[:,1],gnss_ned[:,0],'*',label='GNSS')
pos_ax.plot(ref_ned[:,1],ref_ned[:,0],'*',label='Raw GPS')
pos_ax.plot(filter_ned[:,1],filter_ned[:,0],label='Filter')
pos_ax.set_title('Location')
pos_ax.set_ylabel('North (m)')
pos_ax.set_xlabel('East (m)')
pos_ax.legend(loc='best')
pos_ax.set_aspect('equal')

vel_fig, vel_ax = plt.subplots(3,2, sharex=True)
#vel_ax[0,0].plot(flight_data.time[istart:istop],flight_data.navvn[istart:istop],label='True')
vel_ax[0,0].plot(plot_time[istart:istop],estVEL[istart:istop,0],'r',label='Filter')
vel_ax[0,0].set_ylabel('$V_N$ (m/s)')
vel_ax[0,0].legend()

#vel_ax[1,0].plot(plot_time[istart:istop],flight_data.navve[istart:istop],label='True')
vel_ax[1,0].plot(plot_time[istart:istop],estVEL[istart:istop,1],'r',label='Filter')
vel_ax[1,0].set_ylabel('$V_E$ (m/s)')

#vel_ax[2,0].plot(plot_time[istart:istop],flight_data.navvd[istart:istop],label='True')
vel_ax[2,0].plot(plot_time[istart:istop],estVEL[istart:istop,2],'r',label='Filter')
vel_ax[2,0].set_ylabel('$V_D$ (m/s)')

#vel_ax[0,1].plot(plot_time[istart:istop],flight_data.navvn[istart:istop]-estVEL[istart:istop,0],'r')
vel_ax[0,1].plot(plot_time[istart:istop],nsig*np.sqrt(Pvel[istart:istop,0]),'k')
vel_ax[0,1].plot(plot_time[istart:istop],-nsig*np.sqrt(Pvel[istart:istop,0]),'k')

#vel_ax[1,1].plot(plot_time[istart:istop],flight_data.navve[istart:istop]-estVEL[istart:istop,1],'r')
vel_ax[1,1].plot(plot_time[istart:istop],nsig*np.sqrt(Pvel[istart:istop,1]),'k')
vel_ax[1,1].plot(plot_time[istart:istop],-nsig*np.sqrt(Pvel[istart:istop,1]),'k')

#vel_ax[2,1].plot(plot_time[istart:istop],flight_data.navvd[istart:istop]-estVEL[istart:istop,2],'r')
vel_ax[2,1].plot(plot_time[istart:istop],nsig*np.sqrt(Pvel[istart:istop,2]),'k')
vel_ax[2,1].plot(plot_time[istart:istop],-nsig*np.sqrt(Pvel[istart:istop,2]),'k')

vel_ax[0,0].set_title('Velocity (m/s)')
vel_ax[0,1].set_title('Velocity Error (m/s)')
vel_ax[2,0].set_xlabel('Time (sec)')
vel_ax[2,1].set_xlabel('Time (sec)')

att_fig, att_ax = plt.subplots(3,2, sharex=True)
#att_ax[0,0].plot(plot_time[istart:istop],np.rad2deg(flight_data.phi[istart:istop]),label='True')
att_ax[0,0].plot(plot_time[istart:istop],phi[istart:istop],'r',label='Filter')
att_ax[0,0].set_ylabel(r'$phi$ (deg)')
att_ax[0,0].legend()

#att_ax[1,0].plot(plot_time[istart:istop],np.rad2deg(flight_data.theta[istart:istop]),label='True')
att_ax[1,0].plot(plot_time[istart:istop],theta[istart:istop],'r',label='Filter')
att_ax[1,0].set_ylabel(r'$theta$ (deg)')

#att_ax[2,0].plot(plot_time[istart:istop],np.rad2deg(flight_data.psi[istart:istop]),label='True')
att_ax[2,0].plot(plot_time[istart:istop],psi[istart:istop],'r',label='Filter')
att_ax[2,0].set_ylabel(r'$psi$ (deg)')

att_ax[0,1].plot(plot_time[istart:istop],np.rad2deg(delta_att[istart:istop,0]),'r')
att_ax[0,1].plot(plot_time[istart:istop],nsig*np.rad2deg(np.sqrt(Patt[istart:istop,0])),'k')
att_ax[0,1].plot(plot_time[istart:istop],-nsig*np.rad2deg(np.sqrt(Patt[istart:istop,0])),'k')

att_ax[1,1].plot(plot_time[istart:istop],np.rad2deg(delta_att[istart:istop,1]),'r')
att_ax[1,1].plot(plot_time[istart:istop],nsig*np.rad2deg(np.sqrt(Patt[istart:istop,1])),'k')
att_ax[1,1].plot(plot_time[istart:istop],-nsig*np.rad2deg(np.sqrt(Patt[istart:istop,1])),'k')

att_ax[2,1].plot(plot_time[istart:istop],np.rad2deg(delta_att[istart:istop,2]),'r')
att_ax[2,1].plot(plot_time[istart:istop],nsig*np.rad2deg(np.sqrt(Patt[istart:istop,2])),'k')
att_ax[2,1].plot(plot_time[istart:istop],-nsig*np.rad2deg(np.sqrt(Patt[istart:istop,2])),'k')

att_ax[0,0].set_title('Euler Angle (deg)')
att_ax[0,1].set_title('Attitude Error (deg)')
att_ax[2,0].set_xlabel('Time (sec)')
att_ax[2,1].set_xlabel('Time (sec)')

ab_fig, ab_ax = plt.subplots(3, sharex=True)
#try:
    #ab_ax[0].plot(plot_time[istart:istop],flight_data.ax_bias[istart:istop],label='True')
#except AttributeError:
#    pass
ab_ax[0].plot(plot_time[istart:istop],estAB[istart:istop,0],label='Filter')
ab_ax[0].set_ylabel('$b_{ax}$ (m/s$^2$)')
ab_ax[0].set_title('Accelerometer Bias')

#try:
    #ab_ax[1].plot(plot_time[istart:istop],flight_data.ay_bias[istart:istop],label='True')
#except AttributeError:
#    pass
ab_ax[1].plot(plot_time[istart:istop],estAB[istart:istop,1],label='Filter')
ab_ax[1].set_ylabel('$b_{ay}$ (m/s$^2$)')
ab_ax[1].legend(loc='best')

#try:
    #ab_ax[2].plot(plot_time[istart:istop],flight_data.az_bias[istart:istop],label='True')
#except AttributeError:
#    pass
ab_ax[2].plot(plot_time[istart:istop],estAB[istart:istop,2],label='Filter')
ab_ax[2].set_ylabel('$b_{az}$ (m/s$^2$)')
ab_ax[2].set_xlabel('Time (sec)')

gb_fig, gb_ax = plt.subplots(3, sharex=True)
#try:
    #gb_ax[0].plot(plot_time[istart:istop],np.rad2deg(flight_data.p_bias[istart:istop]),label='True')
#except AttributeError:
#    pass
gb_ax[0].plot(plot_time[istart:istop],np.rad2deg(estGB[istart:istop,0]),label='Filter')
gb_ax[0].set_ylabel('$b_{gx}$ (deg/s)')
gb_ax[0].set_title('Gyro Bias')

#try:
    #gb_ax[1].plot(plot_time[istart:istop],np.rad2deg(flight_data.q_bias[istart:istop]),label='True')
#except AttributeError:
#    pass
gb_ax[1].plot(plot_time[istart:istop],np.rad2deg(estGB[istart:istop,1]),label='Filter')
gb_ax[1].set_ylabel('$b_{gy}$ (deg/s)')
gb_ax[1].legend(loc='best')

#try:
    #gb_ax[2].plot(plot_time[istart:istop],np.rad2deg(flight_data.r_bias[istart:istop]),label='True')
#except AttributeError:
#    pass
gb_ax[2].plot(plot_time[istart:istop],np.rad2deg(estGB[istart:istop,2]),label='Filter')
gb_ax[2].set_ylabel('$b_{gz}$ (deg/s)')
gb_ax[2].set_xlabel('Time (sec)')

cal_fig, cal_gyro = plt.subplots(3, sharex=True)
cal_gyro[0].plot(estGB[istart:istop,3],np.rad2deg(estGB[istart:istop,0]),'*',label='Filter')
cal_gyro[0].set_xlabel('Temp (C)')
cal_gyro[0].set_ylabel('$b_{gx}$ (deg/s)')
cal_gyro[0].set_title('Gyro Bias vs. Temp')
cal_gyro[1].plot(estGB[istart:istop,3],np.rad2deg(estGB[istart:istop,1]),'*',label='Filter')
cal_gyro[1].set_xlabel('Temp (C)')
cal_gyro[1].set_ylabel('$b_{gy}$ (deg/s)')
cal_gyro[1].set_title('Gyro Bias vs. Temp')
cal_gyro[2].plot(estGB[istart:istop,3],np.rad2deg(estGB[istart:istop,2]),'*',label='Filter')
cal_gyro[2].set_xlabel('Temp (C)')
cal_gyro[2].set_ylabel('$b_{gz}$ (deg/s)')
cal_gyro[2].set_title('Gyro Bias vs. Temp')

cal_fig, cal_accel = plt.subplots(3, sharex=True)
cal_accel[0].plot(estAB[istart:istop,3],estAB[istart:istop,0],'*',label='Filter')
cal_accel[0].set_xlabel('Temp (C)')
cal_accel[0].set_ylabel('$b_{ax}$ (m/s$^2$)')
cal_accel[0].set_title('Accel Bias vs. Temp')
cal_accel[1].plot(estAB[istart:istop,3],estAB[istart:istop,1],'*',label='Filter')
cal_accel[1].set_xlabel('Temp (C)')
cal_accel[1].set_ylabel('$b_{ay}$ (m/s$^2$)')
cal_accel[1].set_title('Accel Bias vs. Temp')
cal_accel[2].plot(estAB[istart:istop,3],estAB[istart:istop,2],'*',label='Filter')
cal_accel[2].set_xlabel('Temp (C)')
cal_accel[2].set_ylabel('$b_{az}$ (m/s$^2$)')
cal_accel[2].set_title('Accel Bias vs. Temp')

plt.show()



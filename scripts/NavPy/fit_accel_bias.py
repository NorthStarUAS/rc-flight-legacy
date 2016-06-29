#!/usr/bin/python

import argparse
import os
import sys
import fileinput

import math
import numpy as np
import matplotlib.pyplot as plt

import imucal

argparser = argparse.ArgumentParser(description='fit imu bias data')
argparser.add_argument('--cal-dir', required=True, help='calibration directory')
args = argparser.parse_args()

cal_file = os.path.join(args.cal_dir, "imucal.xml")

bias_files = []
# find all the *-imubias.txt files in the given tree
for path, dirs, files in os.walk(args.cal_dir):
    if files:
        for file in files:
            if file.endswith('-imubias.txt'):
                bias_files.append( os.path.join(path, file) )

# load imu bias data files
bias_data = []
min_temp = None
max_temp = None
for bias_file in bias_files:
    print "loading bias data from: ", bias_file
    f = fileinput.input(bias_file)
    for line in f:
        time, temp, p, q, r, ax, ay, az = line.split()
        bias_data.append( np.array( [time, temp, p, q, r, ax, ay, az]) )
        t = float(temp)
        if min_temp == None or t < min_temp:
            min_temp = t
        if max_temp == None or t > max_temp:
            max_temp = t

print "Temp range: %.1f - %.1f\n" % (min_temp, max_temp)
        
if len(bias_data) == 0:
    print "No bias records loaded, cannot continue..."
    sys.exit()

mag_files = []
# find all the *-mags.txt files in the given tree
for path, dirs, files in os.walk(args.cal_dir):
    if files:
        for file in files:
            if file.endswith('-mags.txt'):
                mag_files.append( os.path.join(path, file) )

# load imu bias data files
mag_data = []
for mag_file in mag_files:
    print "loading mag data from: ", mag_file
    f = fileinput.input(mag_file)
    for line in f:
        rawx, rawy, rawz, calx, caly, calz = line.split()
        if abs(float(rawx)) > 1000:
            print line
        mag_data.append( [rawx, rawy, rawz, calx, caly, calz] )

if len(mag_data) == 0:
    print "No mag records loaded..."


# =========================== Results ===============================
bias_array = np.array(bias_data, dtype=np.float64)
bias_len = bias_array.shape[0]
print "temps:", bias_array[:,1]

mag_array = np.array(mag_data, dtype=np.float64)
mag_len = mag_array.shape[0]
mag_min = mag_array[:,0:3].min() # note: [:,start_index:length]
mag_max = mag_array[:,0:3].max()
print "mag range:", mag_min, mag_max

nosave = imucal.Calibration()
cal = imucal.Calibration()

nosave.p_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,2], 2, full=True )
print "p coefficients = ", nosave.p_bias
print "p residual = ", math.sqrt(res[0]/bias_len) * 180 / math.pi
nosave.q_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,3], 2, full=True )
print "q coefficients = ", nosave.q_bias
print "q residual = ", math.sqrt(res[0]/bias_len) * 180 / math.pi
nosave.r_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,4], 2, full=True )
print "r coefficients = ", nosave.r_bias
print "r residual = ", math.sqrt(res[0]/bias_len) * 180 / math.pi
cal.ax_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,5], 2, full=True )
print "ax coefficients = ", cal.ax_bias
print "ax residual = ", math.sqrt(res[0]/bias_len)
cal.ay_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,6], 2, full=True )
print "ay coefficients = ", cal.ay_bias
print "ay residual = ", math.sqrt(res[0]/bias_len)
cal.az_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,7], 2, full=True )
print "az coefficients = ", cal.az_bias
print "az residual = ", math.sqrt(res[0]/bias_len)

cal.min_temp = min_temp
cal.max_temp = max_temp

cal.hx_fit, res, _, _, _ = np.polyfit( mag_array[:,0], mag_array[:,3], 1, full=True )
print "hx coefficients = ", cal.hx_fit
print "hx residual = ", math.sqrt(res[0]/mag_len)
cal.hy_fit, res, _, _, _ = np.polyfit( mag_array[:,1], mag_array[:,4], 1, full=True )
print "hy coefficients = ", cal.hy_fit
print "hy residual = ", math.sqrt(res[0]/mag_len)
cal.hz_fit, res, _, _, _ = np.polyfit( mag_array[:,2], mag_array[:,5], 1, full=True )
print "hz coefficients = ", cal.hz_fit
print "hz residual = ", math.sqrt(res[0]/mag_len)

cal.save_xml(cal_file)

   
# ============================= PLOTS ======================================

def gen_func( coeffs, min, max, steps ):
    xvals = []
    yvals = []
    step = (max - min) / steps
    func = np.poly1d(coeffs)
    for x in np.arange(min, max+step, step):
        y = func(x)
        xvals.append(x)
        yvals.append(y)
    return xvals, yvals

if False:
    cal_fig, cal_gyro = plt.subplots(3, sharex=True)
    xvals, yvals = gen_func(nosave.p_bias, min_temp, max_temp, 100)
    cal_gyro[0].plot(bias_array[:,1],np.rad2deg(bias_array[:,2]),'r.',xvals,np.rad2deg(yvals),label='Filter')
    cal_gyro[0].set_xlabel('Temp (C)')
    cal_gyro[0].set_ylabel('$b_{gx}$ (deg/s)')
    cal_gyro[0].set_title('Gyro Bias vs. Temp')
    xvals, yvals = gen_func(nosave.q_bias, min_temp, max_temp, 100)
    cal_gyro[1].plot(bias_array[:,1],np.rad2deg(bias_array[:,3]),'g.',xvals,np.rad2deg(yvals), label='Filter')
    cal_gyro[1].set_xlabel('Temp (C)')
    cal_gyro[1].set_ylabel('$b_{gy}$ (deg/s)')
    #cal_gyro[1].set_title('Gyro Bias vs. Temp')
    xvals, yvals = gen_func(nosave.r_bias, min_temp, max_temp, 100)
    cal_gyro[2].plot(bias_array[:,1],np.rad2deg(bias_array[:,4]),'b.',xvals,np.rad2deg(yvals),'g', label='Filter')
    cal_gyro[2].set_xlabel('Temp (C)')
    cal_gyro[2].set_ylabel('$b_{gz}$ (deg/s)')
    #cal_gyro[2].set_title('Gyro Bias vs. Temp')

cal_fig, cal_accel = plt.subplots(3, sharex=True)
xvals, yvals = gen_func(cal.ax_bias, min_temp, max_temp, 100)
cal_accel[0].plot(bias_array[:,1],bias_array[:,5],'r.',xvals,yvals,label='Filter')
cal_accel[0].set_xlabel('Temp (C)')
cal_accel[0].set_ylabel('$b_{ax}$ (m/s^2)')
cal_accel[0].set_title('Accel Bias vs. Temp')
xvals, yvals = gen_func(cal.ay_bias, min_temp, max_temp, 100)
cal_accel[1].plot(bias_array[:,1],bias_array[:,6],'g.',xvals,yvals,label='Filter')
cal_accel[1].set_xlabel('Temp (C)')
cal_accel[1].set_ylabel('$b_{ay}$ (m/s^2)')
#cal_accel[1].set_title('Accel Bias vs. Temp')
xvals, yvals = gen_func(cal.az_bias, min_temp, max_temp, 100)
cal_accel[2].plot(bias_array[:,1],bias_array[:,7],'b.',xvals,yvals,'g',label='Filter')
cal_accel[2].set_xlabel('Temp (C)')
cal_accel[2].set_ylabel('$b_{az}$ (m/s^2)')
#cal_accel[2].set_title('Accel Bias vs. Temp')

mag_fig, mag_fit = plt.subplots(3, sharex=True)
xvals, yvals = gen_func(cal.hx_fit, mag_min, mag_max, 100)
mag_fit[0].plot(mag_array[:,0],mag_array[:,3],'r.',xvals,yvals,label='Filter')
mag_fit[0].set_xlabel('Sensed hx (adc)')
mag_fit[0].set_ylabel('True hx (norm)')
mag_fit[0].set_title('Mag Calibration')
xvals, yvals = gen_func(cal.hy_fit, mag_min, mag_max, 100)
mag_fit[1].plot(mag_array[:,1],mag_array[:,4],'g.',xvals,yvals,label='Filter')
mag_fit[1].set_xlabel('Sensed hy (adc)')
mag_fit[1].set_ylabel('True hx (norm)')
#mag_fit[1].set_title('Accel Bias vs. Temp')
xvals, yvals = gen_func(cal.hz_fit, mag_min, mag_max, 100)
mag_fit[2].plot(mag_array[:,2],mag_array[:,5],'b.',xvals,yvals,'g',label='Filter')
mag_fit[2].set_xlabel('Sensed hz (adc)')
mag_fit[2].set_ylabel('True hx (norm)')
#mag_fit[2].set_title('Accel Bias vs. Temp')

plt.show()



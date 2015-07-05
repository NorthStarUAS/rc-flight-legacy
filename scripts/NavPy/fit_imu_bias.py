#!/usr/bin/python

import os.path
import os
import sys
import fileinput

import math
import numpy as np
import matplotlib.pyplot as plt
import insgps_quat_15state
import navpy

import imucal

np.set_printoptions(precision=5,suppress=True)
plt.close()

def usage():
    print "Usage: " + sys.argv[0] + " <flightdir_root>"

if len(sys.argv) < 2:
    usage()
    sys.exit()

cal_file = sys.argv[1] + "/imucal.xml"

bias_files = []
# find all the imubias.txt files in the given tree
for path, dirs, files in os.walk(sys.argv[1]):
    if files:
        for file in files:
            if file == 'imubias.txt':
                bias_files.append( os.path.join(path,file) )

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

# =========================== Results ===============================
drl = len(bias_data)
drw = len(bias_data[0])

bias_array = np.nan*np.ones((drl,drw))
for i, row in enumerate(bias_data):
    bias_array[i,:] = row[:]

print bias_array[:,1]

nosave = imucal.Calibration()
cal = imucal.Calibration()

nosave.p_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,2], 2, full=True )
print "p coefficients = ", nosave.p_bias
print "p residual = ", math.sqrt(res[0]/drl) * 180 / math.pi
nosave.q_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,3], 2, full=True )
print "q coefficients = ", nosave.q_bias
print "q residual = ", math.sqrt(res[0]/drl) * 180 / math.pi
nosave.r_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,4], 2, full=True )
print "r coefficients = ", nosave.r_bias
print "r residual = ", math.sqrt(res[0]/drl) * 180 / math.pi
cal.ax_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,5], 2, full=True )
print "ax coefficients = ", cal.ax_bias
print "ax residual = ", math.sqrt(res[0]/drl)
cal.ay_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,6], 2, full=True )
print "ay coefficients = ", cal.ay_bias
print "ay residual = ", math.sqrt(res[0]/drl)
cal.az_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,7], 2, full=True )
print "az coefficients = ", cal.az_bias
print "az residual = ", math.sqrt(res[0]/drl)

cal.min_temp = min_temp
cal.max_temp = max_temp

cal.save_xml(cal_file)

   
# ============================= PLOTS ======================================

def gen_func( coeffs, min, max, step ):
    xvals = []
    yvals = []
    func = np.poly1d(coeffs)
    for x in np.arange(min, max, step):
        y = func(x)
        xvals.append(x)
        yvals.append(y)
    return xvals, yvals
    
cal_fig, cal_gyro = plt.subplots(3, sharex=True)
xvals, yvals = gen_func(nosave.p_bias, min_temp, max_temp, 0.1)
cal_gyro[0].plot(bias_array[:,1],np.rad2deg(bias_array[:,2]),'r.',xvals,np.rad2deg(yvals),label='Filter')
cal_gyro[0].set_xlabel('Temp (C)')
cal_gyro[0].set_ylabel('$b_{gx}$ (deg/s)')
cal_gyro[0].set_title('Gyro Bias vs. Temp')
xvals, yvals = gen_func(nosave.q_bias, min_temp, max_temp, 0.1)
cal_gyro[1].plot(bias_array[:,1],np.rad2deg(bias_array[:,3]),'g.',xvals,np.rad2deg(yvals), label='Filter')
cal_gyro[1].set_xlabel('Temp (C)')
cal_gyro[1].set_ylabel('$b_{gy}$ (deg/s)')
cal_gyro[1].set_title('Gyro Bias vs. Temp')
xvals, yvals = gen_func(nosave.r_bias, min_temp, max_temp, 0.1)
cal_gyro[2].plot(bias_array[:,1],np.rad2deg(bias_array[:,4]),'b.',xvals,np.rad2deg(yvals),'g', label='Filter')
cal_gyro[2].set_xlabel('Temp (C)')
cal_gyro[2].set_ylabel('$b_{gz}$ (deg/s)')
cal_gyro[2].set_title('Gyro Bias vs. Temp')

cal_fig, cal_accel = plt.subplots(3, sharex=True)
xvals, yvals = gen_func(cal.ax_bias, min_temp, max_temp, 0.1)
cal_accel[0].plot(bias_array[:,1],bias_array[:,5],'r.',xvals,yvals,label='Filter')
cal_accel[0].set_xlabel('Temp (C)')
cal_accel[0].set_ylabel('$b_{ax}$ (m/s^2)')
cal_accel[0].set_title('Accel Bias vs. Temp')
xvals, yvals = gen_func(cal.ay_bias, min_temp, max_temp, 0.1)
cal_accel[1].plot(bias_array[:,1],bias_array[:,6],'g.',xvals,yvals,label='Filter')
cal_accel[1].set_xlabel('Temp (C)')
cal_accel[1].set_ylabel('$b_{ay}$ (m/s^2)')
cal_accel[1].set_title('Accel Bias vs. Temp')
xvals, yvals = gen_func(cal.az_bias, min_temp, max_temp, 0.1)
cal_accel[2].plot(bias_array[:,1],bias_array[:,7],'b.',xvals,yvals,'g',label='Filter')
cal_accel[2].set_xlabel('Temp (C)')
cal_accel[2].set_ylabel('$b_{az}$ (m/s^2)')
cal_accel[2].set_title('Accel Bias vs. Temp')

plt.show()



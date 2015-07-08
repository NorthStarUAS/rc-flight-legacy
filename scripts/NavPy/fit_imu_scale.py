#!/usr/bin/python

import os.path
import os
import sys
import fileinput

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

scale_files = []
# find all the imucal-scale.xml files in the given tree
for path, dirs, files in os.walk(sys.argv[1]):
    if files:
        for file in files:
            if file == 'imucal-scale.xml':
                scale_files.append( os.path.join(path,file) )

# load imu scale data files
scale_data = []
min_temp = None
max_temp = None
for scale_file in scale_files:
    print "loading scale data from: ", scale_file
    cal = imucal.Calibration(scale_file)
    p_scale = cal.p_scale[2]
    q_scale = cal.q_scale[2]
    r_scale = cal.r_scale[2]
    ax_scale = cal.ax_scale[2]
    ay_scale = cal.ay_scale[2]
    az_scale = cal.az_scale[2]
    row = np.array([p_scale, q_scale, r_scale, ax_scale, ay_scale, az_scale])
    scale_data.append(row)
        
if len(scale_data) == 0:
    print "No scale records loaded, cannot continue..."
    sys.exit()

scale_array = np.nan*np.ones((len(scale_data),6))
for i, row in enumerate(scale_data):
    scale_array[i,:] = row[:]
                             
# =========================== Results ===============================

print "p scale factor = ", np.average(scale_array[:,0])
print "q scale factor = ", np.average(scale_array[:,1])
print "r scale factor = ", np.average(scale_array[:,2])
print "ax scale factor = ", np.average(scale_array[:,3])
print "ay scale factor = ", np.average(scale_array[:,4])
print "az scale factor = ", np.average(scale_array[:,5])

   
# ============================= PLOTS ======================================

drl = len(scale_data)

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
#xvals, yvals = gen_func(cal.p_bias, min_temp, max_temp, 0.1)
cal_gyro[0].plot(scale_array[:,0],label='p')
cal_gyro[0].set_xlabel('Flight #')
cal_gyro[0].set_ylabel('Scale')
cal_gyro[0].set_title('Gyro Scale')
#xvals, yvals = gen_func(cal.q_bias, min_temp, max_temp, 0.1)
cal_gyro[1].plot(scale_array[:,1], label='q')
cal_gyro[1].set_xlabel('Flight #')
cal_gyro[1].set_ylabel('Scale')
cal_gyro[1].set_title('Gyro Scale')
#xvals, yvals = gen_func(cal.r_bias, min_temp, max_temp, 0.1)
cal_gyro[2].plot(scale_array[:,2], label='r')
cal_gyro[2].set_xlabel('Flight #')
cal_gyro[2].set_ylabel('Gyro Scale')
cal_gyro[2].set_title('Gyro Scale')

cal_fig, cal_accel = plt.subplots(3, sharex=True)
#xvals, yvals = gen_func(cal.p_bias, min_temp, max_temp, 0.1)
cal_accel[0].plot(scale_array[:,3],label='ax')
cal_accel[0].set_xlabel('Flight #')
cal_accel[0].set_ylabel('Scale')
cal_accel[0].set_title('Accel Scale')
#xvals, yvals = gen_func(cal.q_bias, min_temp, max_temp, 0.1)
cal_accel[1].plot(scale_array[:,4], label='ay')
cal_accel[1].set_xlabel('Flight #')
cal_accel[1].set_ylabel('Scale')
cal_accel[1].set_title('Accel Scale')
#xvals, yvals = gen_func(cal.r_bias, min_temp, max_temp, 0.1)
cal_accel[2].plot(scale_array[:,5], label='az')
cal_accel[2].set_xlabel('Flight #')
cal_accel[2].set_ylabel('Accel Scale')
cal_accel[2].set_title('Accel Scale')

plt.show()



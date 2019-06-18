#!/usr/bin/python3

import argparse
import os
import sys
import fileinput

import math
import numpy as np
import matplotlib.pyplot as plt

from aurauas_flightdata import imucal
import transformations

argparser = argparse.ArgumentParser(description='fit imu bias data')
argparser.add_argument('--aircraft-dir', required=True, help='top level aircraft flight data directory')
args = argparser.parse_args()

cal_file = os.path.join(args.aircraft_dir, "imucal.json")

bias_files = []
# find all the *-imubias.txt files in the given tree
for path, dirs, files in os.walk(args.aircraft_dir):
    if files:
        for file in files:
            if file == 'imubias.txt':
                bias_files.append( os.path.join(path, file) )

# load imu bias data files
bias_data = []
min_temp = None
max_temp = None
for bias_file in bias_files:
    print("loading bias data from: ", bias_file)
    f = fileinput.input(bias_file)
    for line in f:
        time, temp, p, q, r, ax, ay, az = line.split()
        bias_data.append( np.array( [time, temp, p, q, r, ax, ay, az]) )
        t = float(temp)
        if min_temp == None or t < min_temp:
            min_temp = t
        if max_temp == None or t > max_temp:
            max_temp = t

if len(bias_data) == 0:
    print("No accel bias records loaded...")
else:
    print("Accel bias temp range (C): %.1f - %.1f\n" % (min_temp, max_temp))
        
mag_files = []
# find all the *-mags.txt files in the given tree
for path, dirs, files in os.walk(args.aircraft_dir):
    if files:
        for file in files:
            if file == 'mags.txt':
                mag_files.append( os.path.join(path, file) )

# load imu bias data files
mag_data = []
for mag_file in mag_files:
    print("loading mag data from: ", mag_file)
    f = fileinput.input(mag_file)
    for line in f:
        rawx, rawy, rawz, calx, caly, calz = line.split()
        if abs(float(rawx)) > 1000:
            print(line)
        mag_data.append( [rawx, rawy, rawz, calx, caly, calz] )

if len(mag_data) == 0:
    print("No mag records loaded...")


# =========================== Results ===============================
nosave = imucal.Calibration()
cal = imucal.Calibration()
if not cal.load(cal_file):
    print("Warning: no existing calibration file found, seeding default values.")

# select which components to update
bias_cal = nosave
accel_cal = cal
mag_cal = cal
    
if len(bias_data):
    bias_array = np.array(bias_data, dtype=np.float64)
    bias_len = bias_array.shape[0]
    print("temp range:", bias_array[:,1].min(), bias_array[:,1].max())

    bias_cal.p_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,2], 2, full=True )
    print("p coefficients = ", bias_cal.p_bias)
    if len(res):
        print("p residual = ", math.sqrt(res[0]/bias_len) * 180 / math.pi)
    bias_cal.q_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,3], 2, full=True )
    print("q coefficients = ", bias_cal.q_bias)
    if len(res):
        print("q residual = ", math.sqrt(res[0]/bias_len) * 180 / math.pi)
    bias_cal.r_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,4], 2, full=True )
    print("r coefficients = ", bias_cal.r_bias)
    if len(res):
        print("r residual = ", math.sqrt(res[0]/bias_len) * 180 / math.pi)
    
    accel_cal.ax_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,5], 2, full=True )
    print("ax coefficients = ", accel_cal.ax_bias)
    if len(res):
        print("ax residual = ", math.sqrt(res[0]/bias_len))
    accel_cal.ay_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,6], 2, full=True )
    print("ay coefficients = ", accel_cal.ay_bias)
    if len(res):
        print("ay residual = ", math.sqrt(res[0]/bias_len))
    accel_cal.az_bias, res, _, _, _ = np.polyfit( bias_array[:,1], bias_array[:,7], 2, full=True )
    print("az coefficients = ", accel_cal.az_bias)
    if len(res):
        print("az residual = ", math.sqrt(res[0]/bias_len))

    cal.min_temp = min_temp
    cal.max_temp = max_temp

if len(mag_data):
    mag_array = np.array(mag_data, dtype=np.float64)
    print('mag_array:', mag_array)
    #mag_len = mag_array.shape[0]
    #mag_min = mag_array[:,0:3].min() # note: [:,start_index:length]
    #mag_max = mag_array[:,0:3].max()
    #print("mag range:", mag_min, mag_max

    ideal_array = mag_array[:,3:6]
    sense_array = mag_array[:,0:3]

    affine = transformations.affine_matrix_from_points(sense_array.T, ideal_array.T, usesparse=True)
    mag_cal.mag_affine = affine
    print("affine:")
    np.set_printoptions(precision=10,suppress=True)
    print(affine)
    scale, shear, angles, translate, perspective = transformations.decompose_matrix(affine)
    print(' scale:', scale)
    print(' shear:', shear)
    print(' angles:', angles)
    print(' trans:', translate)
    print(' persp:', perspective)

cal.save(cal_file)

   
# ============================= PLOTS ======================================

def gen_func( coeffs, min, max, steps ):
    if abs(max-min) < 0.0001:
        max = min + 0.1
    xvals = []
    yvals = []
    step = (max - min) / steps
    func = np.poly1d(coeffs)
    for x in np.arange(min, max+step, step):
        y = func(x)
        xvals.append(x)
        yvals.append(y)
    return xvals, yvals

if min_temp == None: min_temp = 27.0
if max_temp == None: max_temp = 27.0

if True:
    cal_fig, cal_gyro = plt.subplots(3, sharex=True)
    xvals, yvals = gen_func(cal.p_bias, min_temp, max_temp, 100)
    cal_gyro[0].plot(bias_array[:,1],np.rad2deg(bias_array[:,2]),'r.',xvals,np.rad2deg(yvals),label='Filter')
    cal_gyro[0].set_xlabel('Temp (C)')
    cal_gyro[0].set_ylabel('$b_{gx}$ (deg/s)')
    cal_gyro[0].set_title('Gyro Bias vs. Temp')
    xvals, yvals = gen_func(cal.q_bias, min_temp, max_temp, 100)
    cal_gyro[1].plot(bias_array[:,1],np.rad2deg(bias_array[:,3]),'g.',xvals,np.rad2deg(yvals), label='Filter')
    cal_gyro[1].set_xlabel('Temp (C)')
    cal_gyro[1].set_ylabel('$b_{gy}$ (deg/s)')
    #cal_gyro[1].set_title('Gyro Bias vs. Temp')
    xvals, yvals = gen_func(cal.r_bias, min_temp, max_temp, 100)
    cal_gyro[2].plot(bias_array[:,1],np.rad2deg(bias_array[:,4]),'b.',xvals,np.rad2deg(yvals),'g', label='Filter')
    cal_gyro[2].set_xlabel('Temp (C)')
    cal_gyro[2].set_ylabel('$b_{gz}$ (deg/s)')
    #cal_gyro[2].set_title('Gyro Bias vs. Temp')

if len(bias_data):
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

if len(mag_data):
    mag_fig, mag_fit = plt.subplots(3, sharex=True)
    mag_fit[0].plot(mag_array[:,0],mag_array[:,3],'r.',label='Filter')
    mag_fit[0].set_xlabel('Sensed hx (adc)')
    mag_fit[0].set_ylabel('True hx (norm)')
    mag_fit[0].set_title('Mag Calibration')
    mag_fit[1].plot(mag_array[:,1],mag_array[:,4],'g.',label='Filter')
    mag_fit[1].set_xlabel('Sensed hy (adc)')
    mag_fit[1].set_ylabel('True hx (norm)')
    #mag_fit[1].set_title('Accel Bias vs. Temp')
    mag_fit[2].plot(mag_array[:,2],mag_array[:,5],'b.',label='Filter')
    mag_fit[2].set_xlabel('Sensed hz (adc)')
    mag_fit[2].set_ylabel('True hx (norm)')
    #mag_fit[2].set_title('Accel Bias vs. Temp')

plt.show()



#!/usr/bin/python

import argparse
import fileinput
import geomag
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import os
import re
from scipy import interpolate
import sys

import navpy

import imucal
import transformations

argparser = argparse.ArgumentParser(description='magcal')
argparser.add_argument('--flight', help='flight log directory')
argparser.add_argument('--sentera', help='sentera flight log directory')
argparser.add_argument('--cal', required=True, help='calibration log directory')
argparser.add_argument('--imu-sn', help='specify imu serial number')
argparser.add_argument('--resample-hz', type=float, default=5.0, help='resample rate (hz)')
argparser.add_argument('--xmin', type=float, help='start time')
argparser.add_argument('--xmax', type=float, help='end time')
argparser.add_argument('--plot', action='store_true', help='plot results.')
args = argparser.parse_args()

g = 9.81

if args.flight:
    # load IMU (+ mag) data
    imu_file = os.path.join(args.flight, "imu-0.txt")
    filter_file = os.path.join(args.flight, "filter-0.txt")
    imucal_file = os.path.join(args.flight, "imucal.json")
    events_file = os.path.join(args.flight, "events.txt")
    imu_data = []
    fimu = fileinput.input(imu_file)
    for line in fimu:
        time, p, q, r, ax, ay, az, hx, hy, hz, temp, status = re.split('[,\s]+', line.rstrip())
        if abs(float(hx)) > 1000:
            print "line:", line
        imu = [ float(time),
                float(p), float(q), float(r),
                float(ax), float(ay), float(az),
                float(hx), float(hy), float(hz),
                float(temp), int(status) ]
        imu_data.append( imu )
    if not len(imu_data):
        print "No imu records loaded, cannot continue..."
        sys.exit()

    # load filter (attitude estimate) data
    filter_data = []
    ffilter = fileinput.input(filter_file)
    for line in ffilter:
        time, lat, lon, alt, vn, ve, vd, phi, the, psi, status = re.split('[,\s]+', line.rstrip())
        filter_data.append( [time, lat, lon, alt, vn, ve, vd, phi, the, psi] )
    if not len(filter_data):
        print "No filter records loaded, cannot continue..."
        quit()

    # load calibration file (for this flight) and back correct for
    # original raw sensor values
    cal = imucal.Calibration()
    cal.load(imucal_file)
    imu_data = cal.back_correct(imu_data)

elif args.sentera:
    imu_file = os.path.join(args.sentera, "imu.csv")
    filter_file = os.path.join(args.sentera, "filter-post.txt")
    imu_data = []
    fimu = fileinput.input(imu_file)
    for line in fimu:
        try:
            time, p, q, r, ax, ay, az, hx, hy, hz, temp = re.split('[,\s]+', line.rstrip())
            mag_orientation = 'newer'
            if mag_orientation == 'older':
                imu_data.append( [float(time)/1000000.0,
                                  -float(p), float(q), -float(r),
                                  -float(ax)*g, float(ay)*g, -float(az)*g,
                                  -float(hx), float(hy), -float(hz),
                                  float(temp)] )
            elif mag_orientation == 'newer':
                imu_data.append( [float(time)/1000000.0,
                                  -float(p), float(q), -float(r),
                                  -float(ax)*g, float(ay)*g, -float(az)*g,
                                  -float(hy), float(hx), float(hz),
                                  float(temp)] )
        except:
            print sys.exc_info()
            print line.rstrip()
    if not len(imu_data):
        print "No imu records loaded, cannot continue..."
        quit()

    filter_data = []
    ffilter = fileinput.input(filter_file)
    for line in ffilter:
        time, lat, lon, alt, vn, ve, vd, phi, the, psi, status = re.split('[,\s]+', line.rstrip())
        filter_data.append( [time, lat, lon, alt, vn, ve, vd, phi, the, psi] )
    if not len(filter_data):
        print "No filter records loaded, cannot continue..."
        quit()
     
imu_data = np.array(imu_data, dtype=np.float64)
x = imu_data[:,0]
imu_hx = interpolate.interp1d(x, imu_data[:,7], bounds_error=False, fill_value=0.0)
imu_hy = interpolate.interp1d(x, imu_data[:,8], bounds_error=False, fill_value=0.0)
imu_hz = interpolate.interp1d(x, imu_data[:,9], bounds_error=False, fill_value=0.0)

filter_data = np.array(filter_data, dtype=float)
x = filter_data[:,0]
filter_alt = interpolate.interp1d(x, filter_data[:,3])
filter_phi = interpolate.interp1d(x, filter_data[:,7])
filter_the = interpolate.interp1d(x, filter_data[:,8])
filter_psi = interpolate.interp1d(x, filter_data[:,9])
alt_min = filter_data[:,3].min()
alt_max = filter_data[:,3].max()
alt_cutoff = alt_min + (alt_max - alt_min) * 0.75
print "Alt range =", alt_min, alt_max, "cutoff =", alt_cutoff

# determine ideal magnetometer in ned coordinates
base_lat = filter_data[0][1]
base_lon = filter_data[0][2]
print "starting at:", lat, lon
gm = geomag.geomag.GeoMag("/usr/lib/python2.7/site-packages/geomag/WMM.COF")
mag = gm.GeoMag(base_lat, base_lon)
mag_ned = np.array( [mag.bx, mag.by, mag.bz] )
print 'raw mag vector:', mag_ned
norm = np.linalg.norm(mag_ned)
print '          norm:', norm
mag_ned /= norm
print '    normalized:', mag_ned

# read the events.txt file to determine when aircraft becomes airborne
# (so we can ignore preflight values.)  Update: also to read the IMU
# serial number.
xmin = None
xmax = None
imu_sn = None
if args.flight:
    fevents = fileinput.input(events_file)
    for line in fevents:
        tokens = line.split()
        if len(tokens) == 3 and tokens[2] == 'airborne' and not xmin:
            xmin = float(tokens[0])
            print "airborne (launch) at t =", xmin
        elif len(tokens) == 5 and tokens[3] == 'complete:' and tokens[4] == 'launch' and not xmax:
            # haven't found a max yet, so update min
            xmin = float(tokens[0])
            print "flight begins at t =", xmin                    
        elif len(tokens) == 4 and float(tokens[0]) > 0 and tokens[2] == 'on' and tokens[3] == 'ground' and not xmax:
            t = float(tokens[0])
            if t - xmin > 60:
                xmax = float(tokens[0])
                print "flight complete at t =", xmax
            else:
                print "warning ignoring sub 1 minute hop"
        elif len(tokens) == 6 and tokens[1] == 'APM2:' and tokens[2] == 'Serial' and tokens[3] == 'Number':
            imu_sn = 'apm2_' + tokens[5]
        elif len(tokens) == 5 and tokens[1] == 'APM2' and tokens[2] == 'Serial' and tokens[3] == 'Number:':
            imu_sn = 'apm2_' + tokens[4]
    if imu_sn:
        print 'IMU s/n:', imu_sn
    else:
        print 'Cannot determine IMU serial number from events.txt file'

if args.imu_sn:
    imu_sn = args.imu_sn
    print 'Using serial number from command line:', imu_sn
    
if not imu_sn:
    print 'Cannot continue without an IMU serial number'
    quit()

if args.xmin:
    xmin = args.xmin
    print 'xmin provided:', xmin
if not xmin:
    print "warning no launch event found"
    xmin = x.min()
    
if args.xmax:
    xmax = args.xmax
    print 'xmax provided:', xmax
if not xmax:
    print "warning no land event found"
    xmax = x.max()

# sanity check in case imu data log ends before events.txt
if xmin < x.min():
    xmin = x.min()
if xmax > x.max():
    xmax = x.max()
    
print "flight range = %.3f - %.3f (%.3f)" % (xmin, xmax, xmax-xmin)
trange = xmax - xmin

sense_data = []
ideal_data = []

for i, x in enumerate( np.linspace(xmin, xmax, trange*args.resample_hz) ):
    alt = filter_alt(x)
    phi = filter_phi(x)
    the = filter_the(x)
    psi = filter_psi(x)
    #print phi, the, psi
    N2B = navpy.angle2dcm(psi, the, phi, input_unit='deg')
    mag_ideal = N2B.dot(mag_ned)
    norm = np.linalg.norm(mag_ideal)
    mag_ideal /= norm
    hx = imu_hx(x)
    hy = imu_hy(x)
    hz = imu_hz(x)
    if abs(hx) > 1000:
        print "oops:", hx, hy, hz
    mag_sense = np.array([hx, hy, hz])
    # print mag_sense
    if abs(psi) < 0.1:
        print mag_sense, mag_ideal
    if args.flight:
        ideal_data.append( mag_ideal[:].tolist() )
        sense_data.append( mag_sense[:].tolist() )
    elif args.sentera and alt >= alt_cutoff:
        ideal_data.append( mag_ideal[:].tolist() )
        sense_data.append( mag_sense[:].tolist() )

ideal_array = np.array(ideal_data, dtype=np.float64)
sense_array = np.array(sense_data, dtype=np.float64)

# compute affine transformation between sensed data and ideal data,
# this is our best estimate of the ideal magnetometer calibration
# using the EKF inertial only solution.

affine = transformations.affine_matrix_from_points(sense_array.T, ideal_array.T)
print "affine:"
np.set_printoptions(precision=10,suppress=True)
print affine
scale, shear, angles, translate, perspective = transformations.decompose_matrix(affine)
print ' scale:', scale
print ' shear:', shear
print ' angles:', angles
print ' trans:', translate
print ' persp:', perspective

# write calibration data points to file (so we can aggregate over
# multiple flights later
if args.flight:
    data_dir = os.path.abspath(args.flight)
elif args.sentera:
    data_dir = os.path.abspath(args.sentera)
    
cal_dir = os.path.join(args.cal, imu_sn)
if not os.path.exists(cal_dir):
    os.makedirs(cal_dir)
filename = os.path.basename(data_dir) + "-mags.txt"
mags_file = os.path.join(cal_dir, filename)
print "mags file:", mags_file
f = open(mags_file, 'w')
for i in range(sense_array.shape[0]):
    f.write( "%.4f %.4f %.4f %.4f %.4f %.4f\n" %
             (sense_array[i][0], sense_array[i][1], sense_array[i][2],
              ideal_array[i][0], ideal_array[i][1], ideal_array[i][2]))
f.close()

# generate affine mapping
af_data = []
for i, s in enumerate(sense_array):
    hs = np.hstack( [s, 1.0] )
    af = np.dot(affine, hs)
    norm = np.linalg.norm(af[:3])
    #print 'affine mapped vector norm:', norm
    #print 'ideal:', ideal_array[i], 'af:', af[:3]
    af[:3] /= norm
    af_data.append(af[:3])
af_array = np.array(af_data)


if args.plot:
    cal_fig, cal_mag = plt.subplots(3, sharex=True)
    cal_mag[0].plot(sense_array[:,0],ideal_array[:,0],'r.',alpha=0.5,label='EKF Estimate')
    cal_mag[0].plot(sense_array[:,0],af_array[:,0],'g.',alpha=0.5,label='Affine Cal')
    cal_mag[0].set_xlabel('(hx) Sensed Mag')
    cal_mag[0].set_ylabel('(hx) Ideal Mag Est')
    cal_mag[0].set_title('Magnetometer Calibration')
    cal_mag[0].legend(loc=0)

    cal_mag[1].plot(sense_array[:,1],ideal_array[:,1],'r.',alpha=0.5,label='hy')
    cal_mag[1].plot(sense_array[:,1],af_array[:,1],'g.',alpha=0.5,label='hy')
    cal_mag[1].set_xlabel('(hy) Sensed Mag')
    cal_mag[1].set_ylabel('(hy) Ideal Mag Est')

    cal_mag[2].plot(sense_array[:,2],ideal_array[:,2],'r.',alpha=0.5,label='hz')
    cal_mag[2].plot(sense_array[:,2],af_array[:,2],'g.',alpha=0.5,label='hz')
    cal_mag[2].set_xlabel('(hz) Sensed Mag')
    cal_mag[2].set_ylabel('(hz) Ideal Mag')

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(ideal_array[:,0], ideal_array[:,1], ideal_array[:,2], c='b',alpha=0.5,label='Ideal Mag (EKF)')
    ax.scatter(af_array[:,0], af_array[:,1], af_array[:,2], c='r',alpha=0.5,label='Calibrated Mag')
    ax.set_xlabel('hx')
    ax.set_ylabel('hy')
    ax.set_zlabel('hz')
    ax.legend(loc=0)
    plt.show()


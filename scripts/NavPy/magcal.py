#!/usr/bin/python

import argparse
import fileinput
import geomag
import numpy as np
import os
import re
from scipy.interpolate import InterpolatedUnivariateSpline

import navpy

argparser = argparse.ArgumentParser(description='magcal')
argparser.add_argument('--flight', help='flight log directory')
argparser.add_argument('--resample-hz', type=float, default=100.0, help='resample rate (hz)')
args = argparser.parse_args()

imu_file = os.path.join(args.flight, "imu-0.txt")
filter_file = os.path.join(args.flight, "filter-0.txt")

imu_data = []
fimu = fileinput.input(imu_file)
for line in fimu:
    time, p, q, r, ax, ay, az, hx, hy, hz, temp, status = re.split('[,\s]+', line.rstrip())
    imu_data.append( [time, p, q, r, ax, ay, az, hx, hy, hz, temp] )
if not len(imu_data):
    print "No imu records loaded, cannot continue..."
    sys.exit()
    
filter_data = []
ffilter = fileinput.input(filter_file)
for line in ffilter:
    time, lat, lon, alt, vn, ve, vd, phi, the, psi, status = re.split('[,\s]+', line.rstrip())
    filter_data.append( [time, lat, lon, alt, vn, ve, vd, phi, the, psi] )
if not len(filter_data):
    print "No imu records loaded, cannot continue..."
    sys.exit()

imu_data = np.array(imu_data, dtype=float)
x = imu_data[:,0]
imu_hx = InterpolatedUnivariateSpline(x, imu_data[:,7])
imu_hy = InterpolatedUnivariateSpline(x, imu_data[:,8])
imu_hz = InterpolatedUnivariateSpline(x, imu_data[:,9])

filter_data = np.array(filter_data, dtype=float)
x = filter_data[:,0]
filter_phi = InterpolatedUnivariateSpline(x, filter_data[:,7])
filter_the = InterpolatedUnivariateSpline(x, filter_data[:,8])
filter_psi = InterpolatedUnivariateSpline(x, filter_data[:,9])

# determine ideal magnetometer in ned coordinates
base_lat = filter_data[0][1]
base_lon = filter_data[0][2]
print "starting at:", lat, lon
gm = geomag.geomag.GeoMag("/usr/lib/python2.7/site-packages/geomag/WMM.COF")
mag = gm.GeoMag(base_lat, base_lon)
mag_ned = np.array( [mag.bx, mag.by, mag.bz] )
norm = np.linalg.norm(mag_ned)
mag_ned /= norm
print mag_ned

xmin = x.min()
#xmin = 1001
xmax = x.max()
print "flight range = %.3f - %.3f (%.3f)" % (xmin, xmax, xmax-xmin)
trange = xmax - xmin

for x in np.linspace(xmin, xmax, trange*args.resample_hz):
    phi = filter_phi(x)
    the = filter_the(x)
    psi = filter_psi(x)
    N2B = navpy.angle2dcm(psi, the, phi, input_unit='deg')
    mag_ideal = N2B.dot(mag_ned)
    norm = np.linalg.norm(mag_ideal)
    mag_ideal /= norm
    hx = imu_hx(x)
    hy = imu_hy(x)
    hz = imu_hz(x)
    mag_sense = np.array([hx, hy, hz])
    norm = np.linalg.norm(mag_sense)
    mag_sense /= norm
    print mag_ideal[0], mag_ideal[1], mag_ideal[2], mag_sense[0], mag_sense[1], mag_sense[2]

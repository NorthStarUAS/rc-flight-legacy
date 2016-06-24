#!/usr/bin/python

import argparse
import fileinput
import geomag
import matplotlib.pyplot as plt
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
events_file = os.path.join(args.flight, "events.txt")
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

# read the events.txt file to determine when aircraft becomes airborne
# (so we can ignore preflight values)
xmin = None
xmax = None
fevents = fileinput.input(events_file)
for line in fevents:
    tokens = line.split()
    if len(tokens) == 3 and tokens[2] == 'airborne' and not xmin:
        xmin = float(tokens[0])
        print "airborne (launch) at t =", xmin
    if len(tokens) == 5 and tokens[3] == 'complete:' and tokens[4] == 'launch' and not xmax:
        # haven't found a max yet, so update min
        xmin = float(tokens[0])
        print "flight begins at t =", xmin                    
    if len(tokens) == 4 and float(tokens[0]) > 0 and tokens[2] == 'on' and tokens[3] == 'ground' and not xmax:
        t = float(tokens[0])
        if t - xmin > 60:
            xmax = float(tokens[0])
            print "flight complete at t =", xmax
        else:
            print "warning ignoring sub 1 minute hop"

if not xmin:
    print "warning no launch event found"
    xmin = x.min()
if not xmax:
    print "warning no land event found"
    xmax = x.max()

print "flight range = %.3f - %.3f (%.3f)" % (xmin, xmax, xmax-xmin)
trange = xmax - xmin

sense_array = np.nan*np.ones((trange*args.resample_hz,3))
ideal_array = np.nan*np.ones((trange*args.resample_hz,3))

for i, x in enumerate( np.linspace(xmin, xmax, trange*args.resample_hz) ):
    phi = filter_phi(x)
    the = filter_the(x)
    psi = filter_psi(x)
    N2B = navpy.angle2dcm(psi, the, phi, input_unit='deg')
    mag_ideal = N2B.dot(mag_ned)
    norm = np.linalg.norm(mag_ideal)
    #mag_ideal /= norm
    hx = imu_hx(x)
    hy = imu_hy(x)
    hz = imu_hz(x)
    mag_sense = np.array([hx, hy, hz])
    norm = np.linalg.norm(mag_sense)
    #mag_sense /= norm
    ideal_array[i,:] = mag_ideal[:]
    sense_array[i,:] = mag_sense[:]
    #print mag_ideal[0], mag_ideal[1], mag_ideal[2], mag_sense[0], mag_sense[1], mag_sense[2]

def gen_func( coeffs, min, max, step ):
    xvals = []
    yvals = []
    func = np.poly1d(coeffs)
    for x in np.arange(min, max+step, step):
        y = func(x)
        xvals.append(x)
        yvals.append(y)
    return xvals, yvals

deg = 1
hx_fit, res, _, _, _ = np.polyfit( sense_array[:,0], ideal_array[:,0], deg, full=True )
hy_fit, res, _, _, _ = np.polyfit( sense_array[:,1], ideal_array[:,1], deg, full=True )
hz_fit, res, _, _, _ = np.polyfit( sense_array[:,2], ideal_array[:,2], deg, full=True )

cal_fig, cal_mag = plt.subplots(3, sharex=True)
xvals, yvals = gen_func(hx_fit, sense_array[:,0].min(), sense_array[:,0].max(), 0.1)
cal_mag[0].plot(sense_array[:,0],ideal_array[:,0],'r.',xvals,yvals,label='hx')
cal_mag[0].set_xlabel('(hx) Sensed Mag Value')
cal_mag[0].set_ylabel('Ideal Mag Value')
cal_mag[0].set_title('Magnetometer Calibration')

xvals, yvals = gen_func(hy_fit, sense_array[:,1].min(), sense_array[:,1].max(), 0.1)
cal_mag[1].plot(sense_array[:,1],ideal_array[:,1],'g.',xvals,yvals,'r',label='Filter')
cal_mag[1].set_xlabel('(hy) Sensed Mag Value')
cal_mag[1].set_ylabel('Ideal Mag Value')

xvals, yvals = gen_func(hz_fit, sense_array[:,2].min(), sense_array[:,2].max(), 0.1)
cal_mag[2].plot(sense_array[:,2],ideal_array[:,2],'b.',xvals,yvals,'g',label='Filter')
cal_mag[2].set_xlabel('(hz) Sensed Mag Value')
cal_mag[2].set_ylabel('Ideal Mag Value')

plt.show()

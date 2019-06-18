#!/usr/bin/python3

import argparse
import csv
import fileinput
import geomag                   # pip3 install geomag
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import math
import numpy as np
import os

import navpy

from aurauas_flightdata import flight_loader, flight_interp, imucal

import transformations

parser = argparse.ArgumentParser(description='magcal')
parser.add_argument('--flight', required=True, help='load specified aura flight log')
parser.add_argument('--imu-sn', help='specify imu serial number')
parser.add_argument('--resample-hz', type=float, default=10.0, help='resample rate (hz)')
parser.add_argument('--xmin', type=float, help='start time')
parser.add_argument('--xmax', type=float, help='end time')
parser.add_argument('--plot', action='store_true', help='plot results.')
args = parser.parse_args()

g = 9.81
r2d = 180.0 / math.pi

print("Loading flight data:", args.flight)
data, flight_format = flight_loader.load(args.flight)
print("imu records:", len(data['imu']))
print("gps records:", len(data['gps']))
print("filter records:", len(data['filter']))
if len(data['imu']) == 0:
    print("not enough data loaded to continue.")
    quit()

dir = os.path.dirname(args.flight)
imucal_json = os.path.join(dir, "imucal.json")
if os.path.exists(imucal_json):
    cal = imucal.Calibration()
    cal.load(imucal_json)
    print('back correcting imu data and biases for original raw values.')
    cal.back_correct(data['imu'], data['filter'])

# this must happen after the back convert
print("Creating interpolation structures..")
interp = flight_interp.InterpolationGroup(data)

cal = imucal.Calibration()
flight_dir = os.path.dirname(args.flight)
cal_file = os.path.join(flight_dir, "imucal.json")
if os.path.exists(cal_file):
    cal.load(cal_file)
    print('back correcting imu data (to get original raw values)')
    cal.back_correct(data['imu'], data['filter'])
    
# read the events-0.csv file to determine when aircraft becomes airborne
# (so we can ignore preflight values.)  Update: also to read the IMU
# serial number.
xmin = None
xmax = None
imu_sn = None
auto_sn = None
if 'event' in data:
    # scan events log for additional info
    for event in data['event']:
        time = event['time']
        msg = event['message']
        #print(time, msg)
        tokens = msg.split()
        if len(tokens) == 2 and tokens[1] == 'airborne' and not xmin:
            print("airborne (launch) at t =", time)
            xmin = time + 30
        elif len(tokens) == 4 and tokens[2] == 'complete:' and tokens[3] == 'launch' and not xmax:
            # haven't found a max yet, so update min
            print("launch complete at t =", time)
            xmin = time + 30
        elif len(tokens) == 3 and time > 0 and tokens[1] == 'on' and tokens[2] == 'ground' and not xmax:
            t = time
            if t - xmin > 60:
                print("flight complete at t =", time)
                xmax = time - 1
            else:
                print("warning ignoring sub 1 minute flight")
        elif len(tokens) == 5 and (tokens[0] == 'APM2:' or tokens[0] == 'Aura3:') and tokens[1] == 'Serial' and tokens[2] == 'Number':
            auto_sn = int(tokens[4])
        elif len(tokens) == 4 and tokens[0] == 'APM2' and tokens[1] == 'Serial' and tokens[2] == 'Number:':
            auto_sn = int(tokens[3])

if args.imu_sn:
    imu_sn = args.imu_sn
    print('Using serial number from command line:', imu_sn)
elif auto_sn:
    imu_sn = auto_sn
    print('Autodetected serial number (AuraUAS):', imu_sn)

if not imu_sn:
    print('Cannot continue without an IMU serial number')
    quit()

if args.xmin:
    xmin = args.xmin
    print('xmin provided:', xmin)
if not xmin:
    print("warning no launch event found")
    xmin = data['imu'][0]['time']
    
if args.xmax:
    xmax = args.xmax
    print('xmax provided:', xmax)
if not xmax:
    print("warning no land event found")
    xmax = data['imu'][-1]['time']

# sanity check in case imu data log ends before events.txt
if xmin < data['imu'][0]['time']:
    xmin = data['imu'][0]['time']
if xmax > data['imu'][-1]['time']:
    xmax = data['imu'][-1]['time']
    
print("flight range = %.3f - %.3f (%.3f)" % (xmin, xmax, xmax-xmin))
trange = xmax - xmin

alt_min = alt_max = data['gps'][0]['alt']
for f in data['gps']:
    if f['alt'] < alt_min: alt_min = f['alt']
    if f['alt'] > alt_max: alt_max = f['alt']
alt_cutoff = alt_min + (alt_max - alt_min) * 0.25
print("Alt range =", alt_min, alt_max, "(for non-AuraUAS formats) cutoff =", alt_cutoff)

# write the IMU temp vs. bias data file

if os.path.isdir(args.flight):
    imubias_file = os.path.join(args.flight, "imubias.txt")
else:
    imubias_file = os.path.join(os.path.dirname(args.flight), "imubias.txt")
print("IMU bias file:", imubias_file)

f = open(imubias_file, 'w')

min_vel = 7.5                   # mps
for filt in data['filter']:
    time = filt['time']
    if time >= xmin and time <= xmax:
        # if we are processing an aura format or we are above
        # cutoff altitude then log entries
        if flight_format == 'aura_csv' or flight_format or 'aura_hdf5' or alt >= alt_cutoff:
            vel = math.sqrt(filt['vn']*filt['vn'] + filt['ve']*filt['ve'])
            if vel >= min_vel:
                interp_imu = interp.query(filt['time'], 'imu')
                f.write( "%.3f %.1f %.4f %.4f %.4f %.4f %.4f %.4f\n" %
                         (filt['time'], interp_imu['temp'],
                          filt['p_bias'], filt['q_bias'], filt['r_bias'],
                          filt['ax_bias'], filt['ay_bias'], filt['az_bias']) )

f.close()

# Now on to the magnetometer ...

# Determine ideal magnetometer vector in ned coordinates

base_lat = data['gps'][0]['lat']
base_lon = data['gps'][0]['lon']
print("flight starts at:", base_lat, base_lon)
#cof_file = os.path.join(geomag.__path__, "WMM.COF")
#gm = geomag.geomag.GeoMag(cof_file)
gm = geomag.geomag.GeoMag()
mag = gm.GeoMag(base_lat, base_lon)
mag_ned = np.array( [mag.bx, mag.by, mag.bz] )
print('  ideal mag vector:', mag_ned)
norm = np.linalg.norm(mag_ned)
print('  norm:', norm)
mag_ned /= norm
print('  normalized:', mag_ned)

# generate array of sensed mag vector vs. rotated ideal mag vector (at
# this point we are fully trusting the EKF attitude estimate and the
# WMM for our calibration.)

sense_data = []
ideal_data = []

for t in np.linspace(xmin, xmax, int(trange*args.resample_hz)):
    filter = interp.query(t, 'filter')
    imu = interp.query(t, 'imu')
    print("imu:", imu, "filter:", filter)
    #psix = filter['psix']
    #psiy = filter['psiy']
    #psi = math.atan2(psiy, psix)
    psi = filter['psi']
    # print phi, the, psi
    N2B = navpy.angle2dcm(psi, filter['the'], filter['phi'], input_unit='rad')
    mag_ideal = N2B.dot(mag_ned)
    norm = np.linalg.norm(mag_ideal)
    mag_ideal /= norm
    hx = imu['hx']
    hy = imu['hy']
    hz = imu['hz']
    if abs(hx) > 1000:
        print("oops:", hx, hy, hz)
    mag_sense = np.array([hx, hy, hz])
    # if abs(psi) < 0.1:
    if flight_format == 'aura_csv' or flight_format == 'aura_hdf5':
        ideal_data.append( mag_ideal[:].tolist() )
        sense_data.append( mag_sense[:].tolist() )
    elif filter['alt'] >= alt_cutoff:
        print(mag_sense, mag_ideal)
        ideal_data.append( mag_ideal[:].tolist() )
        sense_data.append( mag_sense[:].tolist() )

ideal_array = np.array(ideal_data, dtype=np.float64)
sense_array = np.array(sense_data, dtype=np.float64)

# compute affine transformation between sensed data and ideal data,
# this is our best estimate of the ideal magnetometer calibration
# using the EKF inertial only solution.

affine = transformations.affine_matrix_from_points(sense_array.T, ideal_array.T)
print("affine:")
np.set_printoptions(precision=10,suppress=True)
print(affine)
scale, shear, angles, translate, perspective = transformations.decompose_matrix(affine)
print(' scale:', scale)
print(' shear:', shear)
print(' angles:', angles)
print(' trans:', translate)
print(' persp:', perspective)

# write mag calibration data points to file (so we can aggregate over
# multiple flights later

if os.path.isdir(args.flight):
    mags_file = os.path.join(args.flight, "mags.txt")
else:
    mags_file = os.path.join(os.path.dirname(args.flight), "mags.txt")
print("mags file:", mags_file)
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
    #print sense_array[0]
    norms = np.linalg.norm(sense_array, axis=1)
    print('norms:', norms)
    norm_array = np.divide(sense_array, np.matrix(norms).T)
    print('norm_array', norm_array)
    print('norm_array[:,0]', np.array(norm_array[:,0].T)[0])
    print('ideal_array[:,0]', ideal_array[:,0])
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
    ax.scatter(np.array(norm_array[:,0].T)[0], np.array(norm_array[:,1].T)[0], np.array(norm_array[:,2].T)[0], c='g',alpha=0.5,label='Raw Mag')
    ax.scatter(af_array[:,0], af_array[:,1], af_array[:,2], c='r',alpha=0.5,label='Calibrated Mag')
    ax.set_xlabel('hx')
    ax.set_ylabel('hy')
    ax.set_zlabel('hz')
    ax.legend(loc=0)
    plt.show()


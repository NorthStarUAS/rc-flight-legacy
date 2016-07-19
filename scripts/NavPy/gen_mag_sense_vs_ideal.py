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
import transformations

argparser = argparse.ArgumentParser(description='magcal')
argparser.add_argument('--flight', help='flight log directory')
argparser.add_argument('--sentera', help='sentera flight log directory')
argparser.add_argument('--cal', required=True, help='calibration log directory')
argparser.add_argument('--imu-sn', help='specify imu serial number')
argparser.add_argument('--resample-hz', type=float, default=100.0, help='resample rate (hz)')
argparser.add_argument('--plot', action='store_true', help='plot results.')
args = argparser.parse_args()

g = 9.81

if args.flight:
    # load IMU (+ mag) data
    imu_file = os.path.join(args.flight, "imu-0.txt")
    filter_file = os.path.join(args.flight, "filter-0.txt")
    events_file = os.path.join(args.flight, "events.txt")
    imu_data = []
    fimu = fileinput.input(imu_file)
    for line in fimu:
        time, p, q, r, ax, ay, az, hx, hy, hz, temp, status = re.split('[,\s]+', line.rstrip())
        if abs(float(hx)) > 500:
            print "line:", line
        imu_data.append( [time, p, q, r, ax, ay, az, hx, hy, hz, temp] )
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
        print "No imu records loaded, cannot continue..."
        sys.exit()
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
        print "No imu records loaded, cannot continue..."
        quit()
     
imu_data = np.array(imu_data, dtype=np.float64)
x = imu_data[:,0]
print 'imu_data:', imu_data[:,7]
print 'hx range:', imu_data[:,7].min(), imu_data[:,7].max()
print 'hy range:', imu_data[:,8].min(), imu_data[:,8].max()
print 'hz range:', imu_data[:,9].min(), imu_data[:,9].max()
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
norm = np.linalg.norm(mag_ned)
mag_ned /= norm
print mag_ned

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
        print 'IMU s/n: ', imu_sn
    else:
        print 'Cannot determine IMU serial number from events.txt file'

if args.imu_sn:
    imu_sn = args.imu_sn
    print 'Using serial number from command line:', imu_sn
    
if not imu_sn:
    print 'Cannot continue without an IMU serial number'
    quit()
    
if not xmin:
    print "warning no launch event found"
    xmin = x.min()
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
    #mag_ideal /= norm
    hx = imu_hx(x)
    hy = imu_hy(x)
    hz = imu_hz(x)
    if abs(hx) > 500:
        print "oops:", hx, hy, hz
    mag_sense = np.array([hx, hy, hz])
    #print mag_sense
    #norm = np.linalg.norm(mag_sense)
    #ag_sense /= norm
    if args.flight:
        ideal_data.append( mag_ideal[:].tolist() )
        sense_data.append( mag_sense[:].tolist() )
    elif args.sentera and alt >= alt_cutoff:
        ideal_data.append( mag_ideal[:].tolist() )
        sense_data.append( mag_sense[:].tolist() )

    #print mag_ideal[0], mag_ideal[1], mag_ideal[2], mag_sense[0], mag_sense[1], mag_sense[2]
    
ideal_array = np.array(ideal_data, dtype=np.float64)
sense_array = np.array(sense_data, dtype=np.float64)

affine = transformations.affine_matrix_from_points(sense_array.T, ideal_array.T)
print "affine:"
print affine
scale, shear, angles, translate, perspective = transformations.decompose_matrix(affine)
print ' scale:', scale
print ' shear:', shear
print ' angles:', angles
print ' trans:', translate
print ' persp:', perspective

# write calibration data to file (so we can aggregate over
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
for i in range(ideal_array.shape[0]):
    f.write( "%.4f %.4f %.4f %.4f %.4f %.4f\n" %
             (sense_array[i][0], sense_array[i][1], sense_array[i][2],
              ideal_array[i][0], ideal_array[i][1], ideal_array[i][2]))
f.close()

    
def gen_func( coeffs, min, max, steps ):
    print min, max, steps
    step = (max - min) / steps
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

hx_fit_inv, res, _, _, _ = np.polyfit( ideal_array[:,0], sense_array[:,0], deg, full=True )

print hx_fit, hy_fit, hz_fit

ellipse_fit = False
if ellipse_fit:
    # test
    import mag
    m = mag.Magnetometer(F=1.0)
    m.calibrate_bulk(sense_array)
    print "b:", m.b
    print "A_1:", m.A_1

    hx_curt = np.poly1d(hx_fit)
    hy_curt = np.poly1d(hy_fit)
    hz_curt = np.poly1d(hz_fit)
    curt_data = []
    ef_data = []
    for s in sense_data:
        cfx = hx_curt(s[0])
        cfy = hy_curt(s[1])
        cfz = hz_curt(s[2])
        curt = np.array( [cfx, cfy, cfz] )
        norm = np.linalg.norm(curt)
        #curt /= norm
        curt_data.append(curt)

        ef = m.map(s)
        norm = np.linalg.norm(ef)
        #ef /= norm
        ef_data.append(ef)
        #print 's:', s, 'cf:', curt, 'ef:', ef
    curt_array = np.array(curt_data)
    ef_array = np.array(ef_data)

    affine = transformations.affine_matrix_from_points(sense_array.T, ef_array.T)
    print "affine ef:"
    print affine
    scale, shear, angles, translate, perspective = transformations.decompose_matrix(affine)
    print ' scale:', scale
    print ' shear:', shear
    print ' angles:', angles
    print ' trans:', translate
    print ' persp:', perspective

# generate affine mapping
af_data = []
for s in sense_array:
    hs = np.hstack( [s, 1.0] )
    af = np.dot(affine, hs)
    norm = np.linalg.norm(af)
    #af /= norm
    af_data.append(af)
    #print 's:', s, 'af:', af
af_array = np.array(af_data)


if args.plot:
    cal_fig, cal_mag = plt.subplots(3, sharex=True)
    cal_mag[0].plot(sense_array[:,0],ideal_array[:,0],'r.',alpha=0.5,label='EKF Estimate')
    cal_mag[0].plot(sense_array[:,0],ef_array[:,0],'b.',alpha=0.5,label='Ellipsoid Cal')
    cal_mag[0].plot(sense_array[:,0],af_array[:,0],'g.',alpha=0.5,label='Affine Cal')
    xvals, yvals = gen_func(hx_fit, sense_array[:,0].min(), sense_array[:,0].max(), 100)
    cal_mag[0].plot(xvals,yvals,'y',label='EKF Fit')
    cal_mag[0].set_xlabel('(hx) Sensed Mag')
    cal_mag[0].set_ylabel('(hx) Ideal Mag Est')
    cal_mag[0].set_title('Magnetometer Calibration')
    cal_mag[0].legend(loc=0)

    cal_mag[1].plot(sense_array[:,1],ideal_array[:,1],'r.',alpha=0.5,label='hy')
    cal_mag[1].plot(sense_array[:,1],ef_array[:,1],'b.',alpha=0.5,label='hy')
    cal_mag[1].plot(sense_array[:,1],af_array[:,1],'g.',alpha=0.5,label='hy')
    xvals, yvals = gen_func(hy_fit, sense_array[:,1].min(), sense_array[:,1].max(), 100)
    cal_mag[1].plot(xvals,yvals,'y',label='hx')
    cal_mag[1].set_xlabel('(hy) Sensed Mag')
    cal_mag[1].set_ylabel('(hy) Ideal Mag Est')

    cal_mag[2].plot(sense_array[:,2],ideal_array[:,2],'r.',alpha=0.5,label='hz')
    cal_mag[2].plot(sense_array[:,2],ef_array[:,2],'b.',alpha=0.5,label='hz')
    cal_mag[2].plot(sense_array[:,2],af_array[:,2],'g.',alpha=0.5,label='hz')
    xvals, yvals = gen_func(hz_fit, sense_array[:,2].min(), sense_array[:,2].max(), 100)
    cal_mag[2].plot(xvals,yvals,'y',label='hz')
    cal_mag[2].set_xlabel('(hz) Sensed Mag')
    cal_mag[2].set_ylabel('(hz) Ideal Mag')

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    #ax.scatter(sense_array[:,0], sense_array[:,1], sense_array[:,2])
    ax.scatter(curt_array[:,0], curt_array[:,1], curt_array[:,2], c='b',alpha=0.5)
    ax.scatter(af_array[:,0], af_array[:,1], af_array[:,2], c='r',alpha=0.5)
    ax.set_xlabel('hx')
    ax.set_ylabel('hy')
    ax.set_zlabel('hz')
    plt.show()


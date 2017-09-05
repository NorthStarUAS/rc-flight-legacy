#!/usr/bin/python

# Calibrate accels (and magnetometers?) direct from flight log without
# rerunning the EKF in post processing.

import argparse
import csv
import math
import os

from aurauas.flightdata import flight_loader, flight_interp

parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('--flight', required=True, help='load specified aura flight log')
parser.add_argument('--cal', required=True, help='calibration log directory')
parser.add_argument('--imu-sn', help='specify imu serial number')
args = parser.parse_args()

# load the flight data
data, flight_format = flight_loader.load(args.flight, None)
print "imu records:", len(data['imu'])
print "filter records:", len(data['filter'])

# build the interpolation tables
interp = flight_interp.FlightInterpolate()
interp.build(data)

# determine imu serial number (so we can maintain independent
# calibration data bases)
auto_sn = None
if flight_format == 'aura_csv':
    # scan event-0.csv file for additional info
    event_file = os.path.join(args.flight, 'event-0.csv')
    with open(event_file, 'rb') as fevent:
        reader = csv.DictReader(fevent)
        for row in reader:
            time = row['timestamp']
            msg = row['message']
            tokens = msg.split()
            if len(tokens) == 5 and tokens[0] == 'APM2:' and tokens[1] == 'Serial' and tokens[2] == 'Number':
                auto_sn = int(tokens[4])
            elif len(tokens) == 4 and tokens[0] == 'APM2' and tokens[1] == 'Serial' and tokens[2] == 'Number:':
                auto_sn = int(tokens[3])

if args.imu_sn:
    imu_sn = args.imu_sn
    print 'Using serial number from command line:', imu_sn
elif auto_sn:
    imu_sn = auto_sn
    print 'Autodetected serial number (APM2):', imu_sn

# write the output file
cal_dir = os.path.join(args.cal, "apm2_" + str(imu_sn))
if not os.path.exists(cal_dir):
    os.makedirs(cal_dir)

filename = os.path.basename(os.path.abspath(args.flight)) + "-imubias.txt"
cal_file = os.path.join(cal_dir, filename)
print "Calibration file:", cal_file

min_vel = 5 # mps
f = open(cal_file, 'w')

for filt in data['filter']:
    imu_temp = interp.imu_temp(filt.time)
    #print filt.time, imu_temp, filt.p_bias, filt.q_bias, filt.r_bias
    vel = math.sqrt(filt.vn*filt.vn + filt.ve*filt.ve)
    if vel >= min_vel:
        f.write( "%.3f %.1f %.4f %.4f %.4f %.4f %.4f %.4f\n" %
                 (filt.time, imu_temp,
                  filt.p_bias, filt.q_bias, filt.r_bias,
                  filt.ax_bias, filt.ay_bias, filt.az_bias) )

f.close()

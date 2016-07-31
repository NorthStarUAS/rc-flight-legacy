#!/usr/bin/python

import math
import os.path
import sys
import fileinput
import numpy as np
import re

import imucal
import insgps_quat_15state as EKF

def usage():
    print "Usage: " + sys.argv[0] + " <apm_log_file>"
    sys.exit()

if len(sys.argv) != 2:
    usage()

apm_file = sys.argv[1]

deg2rad = math.pi / 180.0

cal = imucal.Calibration()
imu_data = []
gps_data = []
filter_data = []
baro_temp = 0.0
max_speed = 0.0

fapm = fileinput.input(apm_file)
for line in fapm:
    token = line.split(', ')
    if token[0] == "PARM":
        if token[1] == "INS_GYROFFS_X":
            cal.p_bias = np.array( [0.0, 0.0, -float(token[2])] )
        elif token[1] == "INS_GYROFFS_Y":
            cal.q_bias = np.array( [0.0, 0.0, -float(token[2])] )
        elif token[1] == "INS_GYROFFS_Z":
            cal.r_bias = np.array( [0.0, 0.0, float(token[2])] )
        elif token[1] == "INS_ACCOFFS_X":
            cal.ax_bias = np.array( [0.0, 0.0, -float(token[2])] )
        elif token[1] == "INS_ACCOFFS_Y":
            cal.ay_bias = np.array( [0.0, 0.0, -float(token[2])] )
        elif token[1] == "INS_ACCOFFS_Z":
            cal.az_bias = np.array( [0.0, 0.0, float(token[2])] )
        elif token[1] == "INS_ACCSCAL_X":
            cal.ax_scale = np.array( [0.0, 0.0, float(token[2])] )
        elif token[1] == "INS_ACCSCAL_Y":
            cal.ay_scale = np.array( [0.0, 0.0, float(token[2])] )
        elif token[1] == "INS_ACCSCAL_Z":
            cal.az_scale = np.array( [0.0, 0.0, float(token[2])] )
    elif token[0] == "BARO":
        baro_temp = float(token[4])
    elif token[0] == "GPS":
        status = int(token[1])
        tow = float(token[2]) / 1000.0
        sats = int(token[4])
        lat = float(token[6])
        lon = float(token[7])
        alt = float(token[9])
        speed = float(token[10]) # m/s
        if speed > max_speed:
            max_speed = speed
        course = float(token[11])
        vz = float(token[12])
        time = float(token[13]) / 1000.0
        course_rad = math.pi * 0.5 - course * deg2rad
        vx = math.cos(course_rad) * speed;
        vy = math.sin(course_rad) * speed;
        gps = EKF.GPS( float(time), int(status), float(tow),
                       float(lat), float(lon), float(alt),
                       float(vy), float(vx), float(-vz))
        gps_data.append(gps)
    elif token[0] == "IMU":
        time = float(token[1]) / 1000.0
        p = float(token[2]) # was neg
        q = float(token[3]) # was neg
        r = float(token[4])
        ax = float(token[5]) # was neg
        ay = float(token[6]) # was neg
        az = float(token[7])
        imu = EKF.IMU( float(time), 1,
                       float(p), float(q), float(r),
                       float(ax), float(ay), float(az),
                       0.0, 0.0, 0.0,
                       float(baro_temp) )
        imu_data.append(imu)
    # elif token[0] == "IMU":
    #     time = float(token[1]) / 1000.0
    #     p = -float(token[2])
    #     q = -float(token[3])
    #     r = float(token[4])
    #     ax = -float(token[5])
    #     ay = -float(token[6])
    #     az = float(token[7])
    #     imu = EKF.IMU( float(time), 1,
    #                    float(p), float(q), float(r),
    #                    float(ax), float(ay), float(az),
    #                    float(baro_temp) )
    #     imu_data.append(imu)
    elif token[0] == "EKF1":
        time = float(token[1]) / 1000.0
        phi = float(token[2])
        the = float(token[3])
        psi = float(token[4])
        vn = float(token[5])
        ve = float(token[6])
        vd = float(token[7])
        filter = EKF.FILTER( time, 0.0, 0.0, 0.0, vn, ve, vd, phi, the, psi )
        filter_data.append(filter)

print "GPS records: ", len(gps_data)
print "IMU records: ", len(imu_data)
print "Filter records: ", len(filter_data)
print "Max speed: ", max_speed

path, file = os.path.split(apm_file)
#print "path: ", path

cal.save_xml( path + '/imucal.xml' )

fgps = open( path + '/gps.txt', 'w' )
for gps in gps_data:
    line = "%.3f %.10f %.10f %.2f %.4f %.4f %.4f %.3f 8 0\n" % \
           (gps.time, gps.lat, gps.lon, gps.alt, gps.vn, gps.ve, gps.vd,
            gps.tow)
    fgps.write(line)
fgps.close()

fimu = open( path + '/imu.txt', 'w' )
for imu in imu_data:
    line = "%.3f %.4f %.4f %.4f %.4f %.4f %.4f 0.000 0.000 0.000 %.1f 0\n" % \
           (imu.time, imu.p, imu.q, imu.r, imu.ax, imu.ay, imu.az, imu.temp)
    fimu.write(line)
fimu.close()
    
ffilter = open( path + '/filter.txt', 'w' )
for filter in filter_data:
    line = "%.3f %.10f %.10f %.2f %.4f %.4f %.4f %.3f %.3f %.3f 0\n" % \
           (filter.time, filter.lat, filter.lon, filter.alt,
            filter.vn, filter.ve, filter.vd, filter.phi, filter.the,
            filter.psi)
    ffilter.write(line)
ffilter.close()


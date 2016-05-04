#!/usr/bin/python

import argparse
import os
import tempfile

from props import root, getNode

import commands
import current
import parser

vel_node = getNode('/velocity', True)
press_node = getNode('/position/pressure', True)
combined_node = getNode('/position/combined', True)
wind_node = getNode('/filters/wind', True)

def generate_record(id, index):
    if id == parser.GPS_PACKET_V1 or id == parser.GPS_PACKET_V2:
        gps_node = getNode('/sensors/gps[%d]' % index, True)
        data = [ "%.3f" % gps_node.getFloat('timestamp'),
	         "%.10f" % gps_node.getFloat('latitude_deg'),
                 "%.10f" % gps_node.getFloat('longitude_deg'),
                 "%.2f" % gps_node.getFloat('altitude_m'),
	         "%.4f" % gps_node.getFloat('vn_ms'),
                 "%.4f" % gps_node.getFloat('ve_ms'),
                 "%.4f" % gps_node.getFloat('vd_ms'),
	         "%.3f" % gps_node.getFloat('unix_time_sec'),
                 "%d" % gps_node.getInt('satellites'),
                 "%d" % gps_node.getInt('status') ]
        print ','.join(data)
        return ','.join(data)
    elif id == parser.IMU_PACKET_V1 or id == parser.IMU_PACKET_V2 or id == parser.IMU_PACKET_V3:
        imu_node = getNode('/sensors/imu[%d]' % index, True)
        data = [ "%.3f" % imu_node.getFloat('timestamp'),
		 "%.4f" % imu_node.getFloat('p_rad_sec'),
		 "%.4f" % imu_node.getFloat('q_rad_sec'),
		 "%.4f" % imu_node.getFloat('r_rad_sec'),
		 "%.4f" % imu_node.getFloat('ax_mps_sec'),
		 "%.4f" % imu_node.getFloat('ay_mps_sec'),
		 "%.4f" % imu_node.getFloat('az_mps_sec'),
		 "%.3f" % imu_node.getFloat('hx'),
                 "%.3f" % imu_node.getFloat('hy'),
                 "%.3f" % imu_node.getFloat('hz'),
		 "%.1f" % imu_node.getFloat('temp_C'),
                 "%d" % imu_node.getInt('status') ]
        print ','.join(data)
        return ','.join(data)
    elif id == parser.AIRDATA_PACKET_V3 or id == parser.AIRDATA_PACKET_V4 or id == parser.AIRDATA_PACKET_V5:
        airdata_node = getNode('/sensors/airdata[%d]' % index, True)
        data = [ "%.3f" % airdata_node.getFloat('timestamp'),
		 "%.1f" % airdata_node.getFloat('pressure_mbar'),
                 "%.1f" % airdata_node.getFloat('temp_degC'),
		 "%.1f" % vel_node.getFloat('airspeed_smoothed_kt'),
		 "%.2f" % press_node.getFloat('altitude_smoothed_m'),
                 "%.2f" % combined_node.getFloat('altitude_true_m'),
		 "%.2f" % (vel_node.getFloat('pressure_vertical_speed_fps')*60.0),
		 "%.1f" % wind_node.getFloat('wind_dir_deg'),
		 "%.1f" % wind_node.getFloat('wind_speed_kt'),
                 "%.2f" % wind_node.getFloat('pitot_scale_factor'),
                 "%d" % airdata_node.getInt('status') ]
        print ','.join(data)
        return ','.join(data)
    elif id == parser.ACTUATOR_PACKET_V1 or id == parser.ACTUATOR_PACKET_V2:
        filter_node = getNode('/filters/filter[%d]' % index, True)
        record = "%.3f\t%.10f\t%.10f\t%.2f\t%.4f\t%.4f\t%.4f\t%.2f\t%.2f\t%.2f\t%d"
        data = [ "%.3f" % filter_node.getFloat('timestamp'),
		 "%.10f" % filter_node.getFloat('latitude_deg'),
                 "%.10f" % filter_node.getFloat('longitude_deg'),
                 "%.2f" % filter_node.getFloat('altitude_m'),
		 "%.4f" % filter_node.getFloat('vn_ms'),
                 "%.4f" % filter_node.getFloat('ve_ms'),
                 "%.4f" % filter_node.getFloat('vd_ms'),
		 "%.2f" % filter_node.getFloat('roll_deg'),
                 "%.2f" % filter_node.getFloat('pitch_deg'),
                 "%.2f" % filter_node.getFloat('heading_deg'),
		 "%d" % filter_node.getInt('status') ]
        print ','.join(data)
        return ','.join(data)
    elif id == parser.FILTER_PACKET_V1 or id == parser.FILTER_PACKET_V2:
        act_node = getNode('/actuators/actuator[%d]' % index, True)
        record = "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d"
        data = [ "%.3f" % act_node.getFloat('timestamp'),
		 "%.3f" % act_node.getFloatEnum('channel', 0),
		 "%.3f" % act_node.getFloatEnum('channel', 1),
		 "%.3f" % act_node.getFloatEnum('channel', 2),
		 "%.3f" % act_node.getFloatEnum('channel', 3),
		 "%.3f" % act_node.getFloatEnum('channel', 4),
		 "%.3f" % act_node.getFloatEnum('channel', 5),
		 "%.3f" % act_node.getFloatEnum('channel', 6),
		 "%.3f" % act_node.getFloatEnum('channel', 7),
		 "%d" % act_node.getInt('status') ]
        print ','.join(data)
        return ','.join(data)
    elif id == parser.PILOT_INPUT_PACKET_V1 or id == parser.PILOT_INPUT_PACKET_V2:
        pilot_node = getNode('/sensors/pilot_input[%d]' % index, True)
        record = "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d"
        data = [ "%.3f" % pilot_node.getFloat('timestamp'),
		 "%.3f" % pilot_node.getFloat('aileron'),
		 "%.3f" % pilot_node.getFloat('elevator'),
		 "%.3f" % pilot_node.getFloat('throttle'),
		 "%.3f" % pilot_node.getFloat('rudder'),
		 "%.3f" % pilot_node.getFloat('manual'),
		 "%.3f" % pilot_node.getFloatEnum('channel', 5),
		 "%.3f" % pilot_node.getFloatEnum('channel', 6),
		 "%.3f" % pilot_node.getFloatEnum('channel', 7),
		 "%d" % pilot_node.getInt('status') ]
        print ','.join(data)
        return ','.join(data)
    
argparser = argparse.ArgumentParser(description='aura export')
argparser.add_argument('--flight', help='load specified flight log')
argparser.add_argument('--skip-seconds', help='seconds to skip when processing flight log')

args = argparser.parse_args()

data = {}

if args.flight:
    filename = args.flight
    if args.flight.endswith('.gz'):
        (fd, filename) = tempfile.mkstemp()
        command = "zcat " + args.flight + " > " + filename
        print command
        os.system(command)
    try:
        fd = open(filename, 'r')
        full = fd.read()
        if args.flight.endswith('.gz'):
            # remove temporary file name
            os.remove(filename)
    except:
        # eat the expected error
        print "we should be able to ignore the zcat error"

    print "len of decompressed file:", len(full)

    while True:
        #try:
            (id, index) = parser.file_read(full)
            key = '%d-%d' % (id, index)
            record = generate_record(id, index)
            if key in data:
                data[key] += 1
            else:
                data[key] = 1
        #except:
        #    print "end of file"
        #    break
else:
    print "A flight log file must be provided"

for key in sorted(data):
    print key, ":", data[key]


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
        record = "%.3f\t%.10f\t%.10f\t%.2f\t%.4f\t%.4f\t%.4f\t%f\t%d\t%d" % \
		 (gps_node.getFloat('timestamp'),
		  gps_node.getFloat('latitude_deg'),
                  gps_node.getFloat('longitude_deg'),
                  gps_node.getFloat('altitude_m'),
		  gps_node.getFloat('vn_ms'),
                  gps_node.getFloat('ve_ms'),
                  gps_node.getFloat('vd_ms'),
		  gps_node.getFloat('unix_time_sec'),
                  gps_node.getInt('satellites'),
                  gps_node.getInt('status')
                 )
        print record
        return record
    elif id == parser.IMU_PACKET_V1 or id == parser.IMU_PACKET_V2 or id == parser.IMU_PACKET_V3:
        imu_node = getNode('/sensors/imu[%d]' % index, True)
        record = "%.3f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.3f\t%.3f\t%.3f\t%.1f\t%d" % \
                 (imu_node.getFloat('timestamp'),
		  imu_node.getFloat('p_rad_sec'),
		  imu_node.getFloat('q_rad_sec'),
		  imu_node.getFloat('r_rad_sec'),
		  imu_node.getFloat('ax_mps_sec'),
		  imu_node.getFloat('ay_mps_sec'),
		  imu_node.getFloat('az_mps_sec'),
		  imu_node.getFloat('hx'),
                  imu_node.getFloat('hy'),
                  imu_node.getFloat('hz'),
		  imu_node.getFloat('temp_C'),
                  imu_node.getInt('status'))
        print record
        return record
    elif id == parser.AIRDATA_PACKET_V3 or id == parser.AIRDATA_PACKET_V4 or id == parser.AIRDATA_PACKET_V5:
        airdata_node = getNode('/sensors/airdata[%d]' % index, True)
        record = "%.3f\t%.1f\t%.1f\t%.1f\t%.2f\t%.2f\t%.2f\t%.1f\t%.1f\t%.2f\t%d" % \
                 (airdata_node.getFloat('timestamp'),
		  airdata_node.getFloat('pressure_mbar'),
                  airdata_node.getFloat('temp_degC'),
		  vel_node.getFloat('airspeed_smoothed_kt'),
		  press_node.getFloat('altitude_smoothed_m'),
                  combined_node.getFloat('altitude_true_m'),
		  vel_node.getFloat('pressure_vertical_speed_fps')*60.0,
		  wind_node.getFloat('wind_dir_deg'),
		  wind_node.getFloat('wind_speed_kt'),
                  wind_node.getFloat('pitot_scale_factor'),
                  airdata_node.getInt('status')
                 )
        print record
        return record
    elif id == parser.ACTUATOR_PACKET_V1 or id == parser.ACTUATOR_PACKET_V2:
        filter_node = getNode('/filters/filter[%d]' % index, True)
        record = "%.3f\t%.10f\t%.10f\t%.2f\t%.4f\t%.4f\t%.4f\t%.2f\t%.2f\t%.2f\t%d" % \
                 (filter_node.getFloat('timestamp'),
		  filter_node.getFloat('latitude_deg'),
                  filter_node.getFloat('longitude_deg'),
                  filter_node.getFloat('altitude_m'),
		  filter_node.getFloat('vn_ms'),
                  filter_node.getFloat('ve_ms'),
                  filter_node.getFloat('vd_ms'),
		  filter_node.getFloat('roll_deg'),
                  filter_node.getFloat('pitch_deg'),
                  filter_node.getFloat('heading_deg'),
		  filter_node.getInt('status')
                  )
        print record
        return record
    elif id == parser.FILTER_PACKET_V1 or id == parser.FILTER_PACKET_V2:
        act_node = getNode('/actuators/actuator[%d]' % index, True)
        record = "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d" % \
                 (act_node.getFloat('timestamp'),
		  act_node.getFloatEnum('channel', 0),
		  act_node.getFloatEnum('channel', 1),
		  act_node.getFloatEnum('channel', 2),
		  act_node.getFloatEnum('channel', 3),
		  act_node.getFloatEnum('channel', 4),
		  act_node.getFloatEnum('channel', 5),
		  act_node.getFloatEnum('channel', 6),
		  act_node.getFloatEnum('channel', 7),
		  act_node.getInt('status')
                  )
        print record
        return record
    elif id == parser.PILOT_INPUT_PACKET_V1 or id == parser.PILOT_INPUT_PACKET_V2:
        pilot_node = getNode('/sensors/pilot_input[%d]' % index, True)
        record = "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d" % \
                 (pilot_node.getFloat('timestamp'),
		  pilot_node.getFloat('aileron'),
		  pilot_node.getFloat('elevator'),
		  pilot_node.getFloat('throttle'),
		  pilot_node.getFloat('rudder'),
		  pilot_node.getFloat('manual'),
		  pilot_node.getFloatEnum('channel', 5),
		  pilot_node.getFloatEnum('channel', 6),
		  pilot_node.getFloatEnum('channel', 7),
		  pilot_node.getInt('status')
                  )
        print record
        return record
    
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


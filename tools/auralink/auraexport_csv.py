#!/usr/bin/python3

# export more real csv with first line column headers

import argparse
import csv
import datetime
import os
import sys
import tempfile
from tqdm import tqdm

from props import root, getNode

sys.path.append("../../src")
from comms import aura_messages
from comms.packer import packer

import commands
import current
import auraparser

m2nm   = 0.0005399568034557235 # meters to nautical miles

def logical_category(id):
    if id == aura_messages.gps_v2_id or id == aura_messages.gps_v3_id or id == aura_messages.gps_v4_id:
        return 'gps'
    elif id == aura_messages.imu_v3_id or id == aura_messages.imu_v4_id or id == aura_messages.imu_v5_id:
        return 'imu'
    elif id == aura_messages.airdata_v5_id or id == aura_messages.airdata_v6_id or id == aura_messages.airdata_v7_id:
        return 'air'
    elif id == aura_messages.filter_v3_id or id == aura_messages.filter_v4_id or id == aura_messages.filter_v5_id:
        return 'filter'
    elif id == aura_messages.actuator_v2_id or id == aura_messages.actuator_v3_id:
        return 'act'
    elif id == id == aura_messages.pilot_v2_id or id == aura_messages.pilot_v3_id:
        return 'pilot'
    elif id == aura_messages.ap_status_v4_id or id == aura_messages.ap_status_v5_id or id == aura_messages.ap_status_v6_id or id == aura_messages.ap_status_v7_id:
        return 'ap'
    elif id == aura_messages.system_health_v4_id or id == aura_messages.system_health_v5_id or id == aura_messages.system_health_v6_id:
        return 'health'
    elif id == aura_messages.payload_v2_id or id == aura_messages.payload_v3_id:
        return 'payload'
    elif id == aura_messages.event_v1_id or id == aura_messages.event_v2_id:
        return 'event'
    else:
        return 'unknown-packet-id'

# When a binary record of some id is read, it gets parsed into the
# property tree structure.  The following code simple calls the
# appropriate text packer function for the given id to extract the
# same data back out of the property tree and format it as a text
# record.
def generate_record(category, index):
    if category == 'gps':
        return packer.pack_gps_csv(index)
    elif category == 'imu':
        return packer.pack_imu_csv(index)
    elif category == 'air':
        return packer.pack_airdata_csv(index)
    elif category == 'filter':
        return packer.pack_filter_csv(index)
    elif category == 'act':
        return packer.pack_act_csv(index)
    elif category == 'pilot':
        return packer.pack_pilot_csv(index)
    elif category == 'ap':
        return packer.pack_ap_status_csv(index)
    elif category == 'health':
        return packer.pack_system_health_csv(index)
    elif category == 'payload':
        return packer.pack_payload_csv(index)
    elif category == 'event':
        return packer.pack_event_csv(index)

argparser = argparse.ArgumentParser(description='aura export')
argparser.add_argument('flight', help='load specified flight log')
argparser.add_argument('--skip-seconds', help='seconds to skip when processing flight log')

args = argparser.parse_args()

data = {}
master_headers = {}

gps_node = getNode('/sensors/gps', True)
located = False
lon = 0.0
lat = 0.0
sec = 0.0

if args.flight:
    if os.path.isdir(args.flight):
        filename = os.path.join(args.flight, 'flight.dat.gz')
    else:
        filename = args.flight
    print("filename:", filename)
    if filename.endswith('.gz'):
        (fd, filetmp) = tempfile.mkstemp()
        command = 'zcat ' + filename + ' > ' + filetmp
        print(command)
        os.system(command)
        fd = open(filetmp, 'rb')
    else:
        fd = open(filename, 'rb')

    full = fd.read()

    if filename.endswith('.gz'):
        # remove temporary file name
        os.remove(filetmp)
        
    divs = 500
    size = len(full)
    chunk_size = size / divs
    threshold = chunk_size
    print("len of decompressed file:", size)

    print("Parsing log file:")
    t = tqdm(total=size)
    last_counter = 0
    while True:
        try:
            (id, index, counter) = auraparser.file_read(full) 
            t.update(counter-last_counter)
            last_counter = counter
            if not located:
                if gps_node.getInt('satellites') >= 5:
                    lat = gps_node.getFloat('latitude_deg')
                    lon = gps_node.getFloat('longitude_deg')
                    sec = gps_node.getFloat('unix_time_sec')
                    located = True
            current.compute_derived_data()
            category = logical_category(id)
            record, headers = generate_record(category, index)
            key = '%s-%d' % (category, index)
            # print 'key:', key
            if key in data:
                data[key].append(record)
            else:
                data[key] = [ record ]
            if not key in master_headers:
                master_headers[key] = headers
        except:
            t.close()
            print("end of file")
            break
else:
    print("A flight log file must be provided")

output_dir = os.path.dirname(os.path.realpath(filename))

# last recorded time stamp
filter_node = getNode('/filters/filter', True)
status_node = getNode('/status', True)
total_time = filter_node.getFloat('timestamp')
apm2_node = getNode("/sensors/APM2", True)

for key in sorted(data):
    size = len(data[key])
    if total_time > 0.01:
        rate = size / total_time
    else:
        rate = 0.0
    print('%-10s %5.1f/sec (%7d records)' % (key, rate, size))
    if size == 0:
        continue
    filename = os.path.join(output_dir, key + ".csv")
    with open(filename, 'w', encoding='utf8') as csvfile:
        writer = csv.DictWriter( csvfile, fieldnames=master_headers[key] )
        writer.writeheader()
        for row in data[key]:
            writer.writerow(row)
        
print()
print("Total log time: %.1f min" % (total_time / 60.0))
print("Flight timer: %.1f min" % (status_node.getFloat('flight_timer') / 60.0))
print("Autopilot time: %.1f min" % (status_node.getFloat('local_autopilot_timer') / 60.0))
print("Distance flown: %.2f nm (%.2f km)" % (status_node.getFloat('flight_odometer')*m2nm, status_node.getFloat('flight_odometer')*0.001))
print("Battery Usage: %.0f mah" % apm2_node.getInt("extern_current_mah"))
print()

apikey = None
try:
    from os.path import expanduser
    home = expanduser("~")
    f = open(home + '/.forecastio')
    apikey = f.read().rstrip()
except:
    print("you must sign up for a free apikey at forecast.io and insert it as a single line inside a file called ~/.forecastio (with no other text in the file)")

if not apikey:
    print("Cannot lookup weather because no forecastio apikey found.")
elif sec < 1:
    print("Cannot lookup weather because gps didn't report unix time.")
else:
    print()
    #utc = datetime.timezone(0)
    d = datetime.datetime.utcfromtimestamp(sec)
    print(d.strftime("%Y-%m-%d-%H:%M:%S"))

    url = 'https://api.darksky.net/forecast/' + apikey + '/%.8f,%.8f,%.d' % (lat, lon, sec)

    import urllib.request, json
    response = urllib.request.urlopen(url)
    data = json.loads(response.read())
    mph2kt = 0.868976
    mb2inhg = 0.0295299830714
    if 'currently' in data:
        currently = data['currently']
        #for key in currently:
        #    print key, ':', currently[key]
        if 'icon' in currently:
            icon = currently['icon']
            print("Summary:", icon)
        if 'temperature' in currently:
            tempF = currently['temperature']
            tempC = (tempF - 32.0) * 5 / 9
            print("Temp:", "%.1f F" % tempF, "(%.1f C)" % tempC)
        if 'dewPoint' in currently:
            tempF = currently['dewPoint']
            tempC = (tempF - 32.0) * 5 / 9
            print("Dewpoint:", "%.1f F" % tempF, "(%.1f C)" % tempC)
        if 'humidity' in currently:
            hum = currently['humidity']
            print("Humidity:", "%.0f%%" % (hum * 100.0))
        if 'pressure' in currently:
            mbar = currently['pressure']
            inhg = mbar * mb2inhg
            print("Pressure:", "%.2f inhg" % inhg, "(%.1f mbar)" % mbar)
        if 'windSpeed' in currently:
            wind_mph = currently['windSpeed']
            wind_kts = wind_mph * mph2kt
        else:
            wind_mph = 0
            wind_kts = 0
        if 'windBearing' in currently:
            wind_deg = currently['windBearing']
        else:
            wind_deg = 0
        print("Wind %d deg @ %.1f kt (%.1f mph) @ " % (wind_deg, wind_kts, wind_mph, ))
        if 'visibility' in currently:
            vis = currently['visibility']
            print("Visibility:", "%.1f miles" % vis)
        if 'cloudCover' in currently:
            cov = currently['cloudCover']
            print("Cloud Cover:", "%.0f%%" % (cov * 100.0))

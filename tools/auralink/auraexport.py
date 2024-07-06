#!/usr/bin/python3

# export more real csv with first line column headers

import argparse
import datetime
import h5py
import os
import pandas as pd
import sys
import tempfile
from tqdm import tqdm

from PropertyTree import PropertyNode

sys.path.append("../../src")
from comms import ns_messages
from comms.packer import packer

import commands
import current
import auraparser

m2nm = 0.0005399568034557235    # meters to nautical miles

def generate_path(id, index):
    if id in [ns_messages.gps_v3_id, ns_messages.gps_v4_id, ns_messages.gps_v5_id]:
        category = 'gps'
    elif id in [ns_messages.imu_v4_id, ns_messages.imu_v5_id, ns_messages.imu_v6_id]:
        category = 'imu'
    elif id in [ns_messages.airdata_v6_id, ns_messages.airdata_v7_id, ns_messages.airdata_v8_id]:
        category = 'air'
    elif id in [ns_messages.filter_v4_id, ns_messages.filter_v5_id, ns_messages.nav_v6_id]:
        category = 'filter'
    elif id == ns_messages.actuator_v2_id or id == ns_messages.actuator_v3_id:
        category = 'act'
    elif id in [ns_messages.pilot_v2_id, ns_messages.pilot_v3_id, ns_messages.pilot_v4_id]:
        category = 'pilot'
    elif id in [ns_messages.inceptors_v1_id]:
        category = 'inceptors'
    elif id in [ns_messages.ap_status_v6_id, ns_messages.ap_status_v7_id, ns_messages.ap_targets_v1_id]:
        category = 'ap'
    elif id in [ns_messages.system_health_v5_id, ns_messages.system_health_v6_id, ns_messages.status_v7_id]:
        category = 'health'
    elif id == ns_messages.event_v1_id or id == ns_messages.event_v2_id:
        category = 'event'
    else:
        print("Unknown packet id!", id, index)
        path = '/unknown-packet-id'
    if category == 'gps' or category == 'imu' \
       or category == 'air' or category == 'pilot' or category == 'health':
        basepath = '/sensors/' + category
    elif category == 'filter':
        basepath = '/navigation/' + category
    elif category == 'act':
        basepath = '/actuators/' + category
    elif category == 'inceptors':
        basepath = '/inceptors'
    elif category == 'ap':
        basepath = '/autopilot'
    elif category == 'health':
        basepath = '/health'
    elif category == 'event':
        basepath = '/events'
    if index > 0:
        basepath += "-%d" % index
    return category, basepath

# When a binary record of some id is read, it gets parsed into the
# property tree structure.  The following code simple calls the
# appropriate text packer function for the given id to extract the
# same data back out of the property tree and format it as a text
# record.
def generate_record(category, index):
    if category == 'gps':
        return packer.pack_gps_dict(index)
    elif category == 'imu':
        return packer.pack_imu_dict(index)
    elif category == 'air':
        return packer.pack_airdata_dict(index)
    elif category == 'filter':
        return packer.pack_nav_dict(index)
    elif category == 'act':
        return packer.pack_act_dict(index)
    elif category == 'pilot':
        return packer.pack_pilot_dict(index)
    elif category == 'ap':
        return packer.pack_ap_status_dict(index)
    elif category == 'inceptors':
        return packer.pack_inceptors_dict(index)
    elif category == 'health':
        return packer.pack_system_health_dict(index)
    elif category == 'payload':
        return packer.pack_payload_dict(index)
    elif category == 'event':
        return packer.pack_event_dict(index)
    else:
        print("generate_record, unknown category:", category)

argparser = argparse.ArgumentParser(description='aura export')
argparser.add_argument('flight', help='load specified flight log')
argparser.add_argument('--skip-seconds', help='seconds to skip when processing flight log')

args = argparser.parse_args()

data = {}
master_headers = {}

gps_node = PropertyNode('/sensors/gps/0')
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
    t = tqdm(total=size, smoothing=0.05)
    last_counter = 0
    while True:
        try:
            (id, index, counter) = auraparser.file_read(full)
            t.update(counter-last_counter)
            last_counter = counter
            if not located:
                gps_node.pretty_print()
                if gps_node.getInt('num_sats') >= 5:
                    lat = gps_node.getDouble('latitude_deg')
                    lon = gps_node.getDouble('longitude_deg')
                    sec = gps_node.getDouble('unix_time_sec')
                    located = True
            current.compute_derived_data()
            category, path = generate_path(id, index)
            record = generate_record(category, index)
            if path in data:
                data[path].append(record)
            else:
                data[path] = [ record ]
        except IndexError:
            t.close()
            print("end of file")
            break
else:
    print("A flight log file must be provided")

output_dir = os.path.dirname(os.path.realpath(filename))

# last recorded time stamp
filter_node = PropertyNode('/filters/filter/0')
status_node = PropertyNode('/status')
total_time = filter_node.getDouble('timestamp')
filter_node.pretty_print()
apm2_node = PropertyNode("/sensors/APM2")

filename = os.path.join(output_dir, "flight.h5")
f = h5py.File(filename, 'w')

md = f.create_group("/metadata")
md.attrs["format"] = "AuraUAS"
md.attrs["version"] = "2.1"
md.attrs["creator"] = "Curtis L. Olson"
md.attrs["url"] = "https://www.flightgear.org"

for key in sorted(data):
    size = len(data[key])
    if total_time > 0.01:
        rate = size / total_time
    else:
        rate = 0.0
    print('%-10s %5.1f/sec (%7d records)' % (key, rate, size))
    if size == 0:
        continue
    df = pd.DataFrame(data[key])
    df.set_index('timestamp', inplace=True, drop=False)
    for column in df.columns:
        print(key + '/' + column)
        #print(df[column].values)
        #print(type(df[column].values))
        if type(df[column].values[0]) != str:
            f.create_dataset(key + '/' + column,
                             data=df[column].values,
                             compression="gzip", compression_opts=9)
        else:
            # special str handling
            dt = h5py.special_dtype(vlen=str)
            f.create_dataset(key + '/' + column,
                             data=df[column].values, dtype=dt,
                             compression="gzip", compression_opts=9)

f.close()

print()
print("Total log time: %.1f min" % (total_time / 60.0))
print("Flight timer: %.1f min" % (status_node.getDouble("flight_timer") / 60.0))
print("Autopilot timer: %.1f min" % (status_node.getDouble("ap_timer") / 60.0))
print("Throttle timer: %.1f min" % (status_node.getDouble("throttle_timer") / 60.0))
od = status_node.getDouble('odometer_m')
print("Distance flown: %.2f nm (%.2f km)" % (od*m2nm, od*0.001))
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

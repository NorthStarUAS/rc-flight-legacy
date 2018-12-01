#!/usr/bin/python3

import argparse
import csv
import datetime
import fnmatch
import fractions
import math
from matplotlib import pyplot as plt 
import numpy as np
import os
import pyexiv2

from aurauas.flightdata import flight_loader, flight_interp

# helpful constants
d2r = math.pi / 180.0
r2d = 180.0 / math.pi

parser = argparse.ArgumentParser(description='Geotag a set of images from the specified flight data log.')

parser.add_argument('--flight', required=True, help='AuraUAS flight data log directory')
parser.add_argument('--images', required=True, help='Directory containing the images')
parser.add_argument('--hz', default=100, type=int, help='sample rate')
parser.add_argument('--shift-time', type=float, help='manual time shift')
parser.add_argument('--align-first', action='store_true', help='align first in-flight trigger and first image')
parser.add_argument('--no-plot', dest='plot', action='store_false', help='do not show correlation plots')
parser.add_argument('--write', action='store_true', help='update geotags on source images')
parser.set_defaults(plot=True)
args = parser.parse_args()

# load flight data
print('Loading flight data:', args.flight)
data, flight_format = flight_loader.load(args.flight)
interp = flight_interp.FlightInterpolate()
interp.build(data)

# load camera triggers (from the events file)
triggers = []
airborne = False
ap = False
event_file = os.path.join(args.flight, "event-0.csv")
if os.path.exists(event_file):
    with open(event_file, 'r') as fevent:
        reader = csv.DictReader(fevent)
        for row in reader:
            tokens = row['message'].split()
            if len(tokens) == 2 and tokens[0] == 'mission:' and tokens[1] == 'airborne':
                print('airborne @', row['timestamp'])
                airborne = True
            if len(tokens) == 3 and tokens[0] == 'mission:' and tokens[1] == 'on' and tokens[2] == 'ground':
                print('on ground @', row['timestamp'])
                airborne = False
            if len(tokens) == 6 and tokens[0] == 'control:' and tokens[4] == 'on':
                print('ap on @', row['timestamp'])
                ap = True
            if len(tokens) == 6 and tokens[0] == 'control:' and tokens[4] == 'off':
                print('ap off @', row['timestamp'])
                ap = False
            if airborne and ap and len(tokens) == 4 and tokens[0] == 'camera:':
                triggers.append([ float(row['timestamp']),
                                  float(tokens[1]),
                                  float(tokens[2]),
                                  float(tokens[3]) ])

# generate trigger signal
if len(triggers) < 1:
    print('No camera trigger events found')
    quit()
min_trig = triggers[0][0]
max_trig = triggers[-1][0]
print(min_trig, max_trig, 'total triggers:', len(triggers))
signal = [0.0] * int((max_trig-min_trig)*args.hz + 1)
for t in triggers:
    time = t[0]
    index = int((time-min_trig) * args.hz)
    signal[index] = 1
trigger_signal = np.array(signal)

# load list of images
files = []
for file in os.listdir(args.images):
    if fnmatch.fnmatch(file, '*.jpg') or fnmatch.fnmatch(file, '*.JPG'):
        files.append(file)
files.sort()
images = []

# read image exif timestamp (and convert to unix seconds)
for f in files:
    name = os.path.join(args.images, f)
    print(name)
    exif = pyexiv2.ImageMetadata(name)
    exif.read()
    # print exif.exif_keys
    strdate, strtime = str(exif['Exif.Image.DateTime'].value).split()
    year, month, day = strdate.split('-')
    hour, minute, second = strtime.split(':')
    d = datetime.date(int(year), int(month), int(day))
    t = datetime.time(int(hour), int(minute), int(second))
    dt = datetime.datetime.combine(d, t) 
    unixtime = dt.strftime('%s')
    images.append([float(unixtime), name])

# sort images by timestamp
images = sorted(images, key=lambda fields: fields[0])

# generate images signal
if len(images) < 1:
    print('No images found')
    quit()
min_image = float(images[0][0])
max_image = float(images[-1][0])
print(min_image, max_image, 'num images:', len(images))
signal = [0.0] * int((max_image-min_image)*args.hz + 1)
width = 4.0                       # seconds
for i in images:
    time = i[0]
    index = int((time-min_image) * args.hz)
    for i in range(-int(args.hz*width*0.5), int(args.hz*width*0.5) + 1):
        x = float(i) * math.pi/(args.hz*width)
        #print i, x, math.cos(x)
        if (index + i) >= 0 and (index + i) < len(signal):
            val = math.cos(x) * 0.99
            # max
            if val > signal[index + i]:
                signal[index + i] = val
            # sum
            # signal[index + i] += val
images_signal = np.array(signal)

print("running correlation between image and trigger signals")
#ycorr = np.correlate(trigger_signal, images_signal, mode='full')
ycorr = np.convolve(trigger_signal, images_signal, mode='valid')

if args.shift_time:
    # override correlation result
    shift = args.shift_time
    max_index = shift * args.hz
elif args.align_first:
    max_index = 0
else:
    max_index = np.argmax(ycorr)
    
print("len triggers:", len(trigger_signal))
print("len images:", len(images_signal))
print("max index:", max_index)
shift = float(max_index) / args.hz
print("relative time shift: %.f" % (shift))
if len(trigger_signal) < len(images_signal):
    # should be ok
    pass
else:
    print("I DON'T KNOW WHAT TO DO HERE ... FIGURE IT OUT!  But might be ok?")

if args.plot:
    plt.figure(1)
    plt.plot(np.array(range(0, len(trigger_signal))) + max_index,
             trigger_signal, label='triggers')
    plt.plot(np.array(range(0, len(images_signal))),
             images_signal, label='images')
    plt.legend()

    plt.figure(2)
    plt.plot(ycorr)

    plt.show()

class Fraction(fractions.Fraction):
    """Only create Fractions from floats.

    >>> Fraction(0.3)
    Fraction(3, 10)
    >>> Fraction(1.1)
    Fraction(11, 10)
    """

    def __new__(cls, value, ignore=None):
        """Should be compatible with Python 2.6, though untested."""
        return fractions.Fraction.from_float(value).limit_denominator(99999)

def dms_to_decimal(degrees, minutes, seconds, sign=' '):
    """Convert degrees, minutes, seconds into decimal degrees.

    >>> dms_to_decimal(10, 10, 10)
    10.169444444444444
    >>> dms_to_decimal(8, 9, 10, 'S')
    -8.152777777777779
    """
    return (-1 if sign[0] in 'SWsw' else 1) * (
        float(degrees)        +
        float(minutes) / 60   +
        float(seconds) / 3600
    )

def decimal_to_dms(decimal):
    """Convert decimal degrees into degrees, minutes, seconds.

    >>> decimal_to_dms(50.445891)
    [Fraction(50, 1), Fraction(26, 1), Fraction(113019, 2500)]
    >>> decimal_to_dms(-125.976893)
    [Fraction(125, 1), Fraction(58, 1), Fraction(92037, 2500)]
    """
    remainder, degrees = math.modf(abs(decimal))
    remainder, minutes = math.modf(remainder * 60)
    return [Fraction(n) for n in (degrees, minutes, remainder * 60)]

def closest_trigger(time):
    index = 0
    diff = None
    for i, t in enumerate(triggers):
        d = time - t[0]
        if diff == None or (d >= 0 and d < diff):
            diff = d
            index = i
    print('  closest trigger:', triggers[index], 'diff:', diff)
    if diff != None and diff >= 0 and diff <= 10:
        return index
    else:
        return None

# traverse the image list and create output csv file
output_file = os.path.join(args.images, 'pix4d.csv')
with open(output_file, 'w') as csvfile:
    writer = csv.DictWriter( csvfile,
                             fieldnames=['File Name',
                                         'Lat (decimal degrees)',
                                         'Lon (decimal degrees)',
                                         'Alt (meters MSL)',
                                         'Roll (decimal degrees)',
                                         'Pitch (decimal degrees)',
                                         'Yaw (decimal degrees)'] )
    writer.writeheader()
    for i in images:
        time = i[0]
        image = i[1]
        print(image)
        log_time = time - min_image - shift + min_trig
        print(image, ' trigger:', log_time)
        index = closest_trigger(log_time)
        if index == None:
            print('no trigger event found for this image')
            continue
        trigger = triggers[index]
        trigger_time = trigger[0]
        lat_deg = interp.filter_lat(trigger_time)*r2d
        lon_deg = interp.filter_lon(trigger_time)*r2d
        alt_m = interp.filter_alt(trigger_time)
        roll_deg = interp.filter_phi(trigger_time)*r2d
        pitch_deg = interp.filter_the(trigger_time)*r2d
        psix = interp.filter_psix(trigger_time)
        psiy = interp.filter_psiy(trigger_time)
        yaw_deg = math.atan2(psiy, psix)*r2d
        if yaw_deg < 0: yaw_deg += 360.0
        writer.writerow( { 'File Name': os.path.basename(image),
                           'Lat (decimal degrees)': "%.10f" % lat_deg,
                           'Lon (decimal degrees)': "%.10f" % lon_deg,
                           'Alt (meters MSL)': "%.2f" % alt_m,
                           'Roll (decimal degrees)': "%.2f" % roll_deg,
                           'Pitch (decimal degrees)': "%.2f" % pitch_deg,
                           'Yaw (decimal degrees)': "%.2f" % yaw_deg } )

# traverse the image list and geotag
for i in images:
    time = i[0]
    image = i[1]
    print(image,)
    print('  unix:', time)
    print('  min_image:', min_image)
    print('  shift:', shift)
    print('  min_trigger:', min_trig)
    log_time = time - min_image - shift + min_trig
    print(' trigger:', time - min_image - shift + min_trig)
    index = closest_trigger(log_time)
    if index == None:
        print('no trigger event found for this image')
        continue
    trigger = triggers[index]
    trigger_time = trigger[0]
    lat = trigger[1]
    lon = trigger[2]
    msl = trigger[3]
    
    lat_deg = interp.filter_lat(time)*r2d
    lon_deg = interp.filter_lon(time)*r2d
    altitude_m = interp.filter_alt(time)
    roll_deg = interp.filter_phi(time)*r2d
    pitch_deg = interp.filter_the(time)*r2d
    psix = interp.filter_psix(time)
    psiy = interp.filter_psiy(time)
    yaw_deg = math.atan2(psiy, psix)*r2d

    print(lat, interp.filter_lat(trigger_time)*r2d, lon, interp.filter_lon(trigger_time)*r2d, msl, interp.filter_alt(trigger_time))

    # update geotag in exif data
    exif = pyexiv2.ImageMetadata(image)
    exif.read()
    GPS = 'Exif.GPSInfo.GPS'
    exif[GPS + 'AltitudeRef']  = '0' if msl >= 0 else '1'
    exif[GPS + 'Altitude']     = Fraction(msl)
    exif[GPS + 'Latitude']     = decimal_to_dms(lat)
    exif[GPS + 'LatitudeRef']  = 'N' if lat >= 0 else 'S'
    exif[GPS + 'Longitude']    = decimal_to_dms(lon)
    exif[GPS + 'LongitudeRef'] = 'E' if lon >= 0 else 'W'
    exif[GPS + 'MapDatum']     = 'WGS-84'
    if args.write:
        exif.write()
    else:
        print('    dry run, not writing geotag.')


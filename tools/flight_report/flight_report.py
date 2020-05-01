#!/usr/bin/python3

"""run_filter.py

Run a flight data set through a filter and output a few simple plots
Author: Curtis L. Olson, University of Minnesota
"""

import argparse
import datetime
import math
from matplotlib import pyplot as plt
import matplotlib.transforms
#import mpld3
import numpy as np
import os
import pandas as pd
from tqdm import tqdm

from aurauas_flightdata import flight_loader, flight_interp

parser = argparse.ArgumentParser(description='nav filter')
parser.add_argument('flight', help='flight data log')
parser.add_argument('--wind-time', type=float, help='force a wind re-estimate with this time factor.')
args = parser.parse_args()

r2d = 180.0 / math.pi
d2r = math.pi / 180.0
m2nm = 0.0005399568034557235    # meters to nautical miles
mps2kt = 1.94384               # m/s to kts

path = args.flight
data, flight_format = flight_loader.load(path)

print("imu records:", len(data['imu']))
imu_dt = (data['imu'][-1]['time'] - data['imu'][0]['time']) \
    / float(len(data['imu']))
print("imu dt: %.3f" % imu_dt)
print("gps records:", len(data['gps']))
if 'air' in data:
    print("airdata records:", len(data['air']))
if len(data['imu']) == 0 and len(data['gps']) == 0:
    print("not enough data loaded to continue.")
    quit()

# make data frames for easier plotting
df0_imu = pd.DataFrame(data['imu'])
df0_imu.set_index('time', inplace=True, drop=False)
df0_gps = pd.DataFrame(data['gps'])
df0_gps.set_index('time', inplace=True, drop=False)
df0_nav = pd.DataFrame(data['filter'])
df0_nav.set_index('time', inplace=True, drop=False)
df0_air = pd.DataFrame(data['air'])
df0_air.set_index('time', inplace=True, drop=False)
if 'health' in data:
    df0_health = pd.DataFrame(data['health'])
    df0_health.set_index('time', inplace=True, drop=False)
if 'act' in data:
    df0_act = pd.DataFrame(data['act'])
    df0_act.set_index('time', inplace=True, drop=False)
if 'pilot' in data:
    df0_pilot = pd.DataFrame(data['pilot'])
    df0_pilot.set_index('time', inplace=True, drop=False)

launch_sec = None
mission = None
land_sec = None
odometer = 0.0
flight_time = 0.0
log_time = data['imu'][-1]['time'] - data['imu'][0]['time']

# Scan events log if it exists
if 'event' in data:
    messages = []
    for event in data['event']:
        time = event['time']
        msg = event['message']
        # print(time, msg)
        tokens = msg.split()
        if len(tokens) == 2 and tokens[1] == 'airborne' and not launch_sec:
            print("airborne (launch) at t =", time)
            launch_sec = time
        elif len(tokens) == 4 and tokens[2] == 'complete:' and tokens[3] == 'launch' and not mission:
            # haven't found a mission start yet, so update time
            print("launch complete at t =", time)
            mission = time
        elif len(tokens) == 3 and time > 0 and tokens[1] == 'on' and tokens[2] == 'ground' and not land_sec:
            t = time
            if t - launch_sec > 60:
                print("flight complete at t =", time)
                land_sec = time
            else:
                print("warning ignoring sub 1 minute flight")
        elif len(tokens) == 5 and (tokens[0] == 'APM2:' or tokens[0] == 'Aura3:') and tokens[1] == 'Serial' and tokens[2] == 'Number':
            auto_sn = int(tokens[4])
        elif len(tokens) == 4 and tokens[0] == 'APM2' and tokens[1] == 'Serial' and tokens[2] == 'Number:':
            auto_sn = int(tokens[3])

regions = []
if 'event' in data:
    # make time regions from event log
    label = "n/a"
    startE = 0.0
    for event in data['event']:
        if event['message'][:7] == "Test ID":
            label = event['message']
        if event['message'] == "Excitation Start":
            startE = event['time']
        if event['message'] == "Excitation End":
            regions.append( [startE, event['time'], label] )
    
# Iterate through the flight and collect some stats
print("Collecting flight stats:")
in_flight = False
ap_time = 0.0
ap_enabled = False
last_time = 0.0
total_mah = 0.0
airborne = []
startA = 0.0
iter = flight_interp.IterateGroup(data)
for i in tqdm(range(iter.size())):
    record = iter.next()
    imu = record['imu']
    if 'gps' in record:
        gps = record['gps']
    if 'air' in record:
        air = record['air']
        if startA == 0.0 and air['airspeed'] >= 15:
            startA = air['time']
        if startA > 0.0 and air['airspeed'] <= 10:
            if air['time'] - startA >= 10.0:
                airborne.append([startA, air['time'], "Airborne"])
            startA = 0.0
        if not in_flight and air['airspeed'] >= 15:
            in_flight = True
        elif in_flight and air['airspeed'] <= 10:
            in_flight = False
    if 'pilot' in record:
        pilot = record['pilot']
        if pilot['auto_manual'] > 0.0:
            ap_enabled = True
        else:
            ap_enable = False
    if 'health' in record:
        health = record['health']
        if 'total_mah' in health:
            total_mah = health['total_mah']
    if 'filter' in record:
        nav = record['filter']
        current_time = nav['time']
        dt = current_time - last_time
        last_time = current_time
        if in_flight:
            flight_time += dt
            vn = nav['vn']
            ve = nav['ve']
            vel_ms = math.sqrt(vn*vn + ve*ve)
            odometer += vel_ms * dt
        if in_flight and ap_enabled:
            ap_time += dt
# catch a truncated flight log
if startA > 0.0:
    airborne.append([startA, air['time'], "Airborne"])
            
# Generate markdown report
f = open("report.md", "w")

plotname = os.path.basename(args.flight.rstrip('/'))

f.write("# Flight Report\n")
f.write("\n")
f.write("## Summary\n")
f.write("- File: %s\n" % plotname)
d = datetime.datetime.utcfromtimestamp( data['gps'][0]['unix_sec'] )
f.write("- Date: %s (UTC)\n" % d.strftime("%Y-%m-%d %H:%M:%S"))
f.write("- Log time: %.1f minutes\n" % (log_time / 60.0))
f.write("- Flight time: %.1f minutes\n" % (flight_time / 60.0))
if ap_time > 0.0:
    f.write("- Autopilot time: %.1f minutes\n" % (ap_time / 60.0))
if odometer > 0.0:
    f.write("- Flight distance: %.2f nm (%.2f km)\n" % (odometer*m2nm, odometer/1000.0))
if odometer > 0.0 and flight_time > 0.0:
    gs_mps = odometer / flight_time
    f.write("- Average ground speed: %.1f kts (%.1f m/s)\n" % (gs_mps * mps2kt, gs_mps))
if total_mah > 0.0:
    f.write("- Total Battery: " + "%0f" % total_mah + " (mah)\n")
f.write("\n")

# Weather Summary

apikey = None
try:
    from os.path import expanduser
    home = expanduser("~")
    fio = open(home + '/.forecastio')
    apikey = fio.read().rstrip()
    fio.close()
except:
    print("you must sign up for a free apikey at forecast.io and insert it as a single line inside a file called ~/.forecastio (with no other text in the file)")

unix_sec = data['gps'][0]['unix_sec']
lat = data['gps'][0]['lat']
lon = data['gps'][0]['lon']

if not apikey:
    print("Cannot lookup weather because no forecastio apikey found.")
elif unix_sec < 1:
    print("Cannot lookup weather because gps didn't report unix time.")
else:
    f.write("## Weather\n")
    d = datetime.datetime.utcfromtimestamp(unix_sec)
    print(d.strftime("%Y-%m-%d-%H:%M:%S"))

    url = 'https://api.darksky.net/forecast/' + apikey + '/%.8f,%.8f,%.d' % (lat, lon, unix_sec)

    import urllib.request, json
    response = urllib.request.urlopen(url)
    wx = json.loads(response.read())
    mph2kt = 0.868976
    mb2inhg = 0.0295299830714
    if 'currently' in wx:
        currently = wx['currently']
        #for key in currently:
        #    print key, ':', currently[key]
        if 'icon' in currently:
            icon = currently['icon']
            f.write("- Conditions: " + icon + "\n")
        if 'temperature' in currently:
            tempF = currently['temperature']
            tempC = (tempF - 32.0) * 5 / 9
            f.write("- Temperature: %.1f F" % tempF + " (%.1f C)" % tempC + "\n")
        else:
            tempF = 0.0
            tempC = 0.0
        if 'dewPoint' in currently:
            dewF = currently['dewPoint']
            dewC = (dewF - 32.0) * 5 / 9
            f.write("- Dewpoint: %.1f F" % dewF + " (%.1f C)" % dewC + "\n")
        else:
            dewF = 0.0
            dewC = 0.0
        if 'humidity' in currently:
            hum = currently['humidity']
            f.write("- Humidity: %.0f%%" % (hum * 100.0) + "\n")
        if 'pressure' in currently:
            mbar = currently['pressure']
            inhg = mbar * mb2inhg
        else:
            mbar = 0
            inhg = 11.11
        f.write("- Pressure: %.2f inhg" % inhg + " (%.1f mbar)" % mbar + "\n")
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
        f.write("- Wind %d deg @ %.1f kt (%.1f mph)" % (wind_deg, wind_kts, wind_mph) + "\n")
        if 'visibility' in currently:
            vis = currently['visibility']
            f.write("- Visibility: %.1f miles" % vis + "\n")
        else:
            vis = 10
        if 'cloudCover' in currently:
            cov = currently['cloudCover']
            f.write("- Cloud Cover: %.0f%%" % (cov * 100.0) + "\n")
        f.write("- METAR: KXYZ " + d.strftime("%d%H%M") + "Z" +
                " %03d%02dKT" % (round(wind_deg/10)*10, wind_kts) +
                " " + ("%.1f" % vis).rstrip('0').rstrip(".") + "SM" +
                " " + ("%.0f" % tempC).replace('-', 'M') + "/" +
                ("%.0f" % dewC).replace('-', 'M') +
                " A%.0f=\n" % (inhg*100)
        )
    f.write("\n")

# end of written report
f.close()


# add a shaded time region(s) to plot
blend = matplotlib.transforms.blended_transform_factory
def add_regions(plot, regions):
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b',
              '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    trans = blend(plot.transData, plot.transAxes)
    for i, region in enumerate(regions):
        plot.axvspan(region[0], region[1], color=colors[i % len(colors)],
                     alpha=0.25)
        plot.text(region[0], 0.5, region[2], transform=trans,
                  verticalalignment='center',
                  rotation=90, color=colors[i % len(colors)])


        
if not 'wind_dir' in data['air'][0] or args.wind_time:
    # run a quick wind estimate
    import wind
    w = wind.Wind()
    winds = w.estimate(data, args.wind_time)
    df1_wind = pd.DataFrame(winds)
    time = df1_wind['time']
    wind_dir = df1_wind['wind_deg']
    wind_speed = df1_wind['wind_kt']
    pitot_scale = df1_wind['pitot_scale']
else:
    time = df0_air['time']
    wind_dir = df0_air['wind_dir']
    wind_speed = df0_air['wind_speed']
    pitot_scale = df0_air['pitot_scale']
    
wind_fig, (ax0, ax1) = plt.subplots(2, 1, sharex=True)
ax0.set_title("Winds Aloft")
ax0.set_ylabel("Heading (degrees)", weight='bold')
ax0.plot(time, wind_dir)
add_regions(ax0, airborne)
ax0.grid()
ax1.set_xlabel("Time (secs)", weight='bold')
ax1.set_ylabel("Speed (kts)", weight='bold')
ax1.plot(time, wind_speed, label="Wind Speed")
ax1.plot(time, pitot_scale, label="Pitot Scale")
add_regions(ax1, airborne)
ax1.grid()
ax1.legend()

# How bad are your magnetometers?
import mags
result = mags.estimate(data)
df1_mags = pd.DataFrame(result)
plt.figure()
plt.title("Magnetometer Norm vs. Throttle")
plt.plot(df1_mags['time'], df1_mags['throttle'])
avg = df1_mags['mag_norm'].mean()
plt.plot(df1_mags['time'], df1_mags['mag_norm']/avg)
plt.xlabel("Time (secs)", weight='bold')
plt.ylabel("Mag Norm", weight='bold')
plt.legend()
plt.grid()

plt.figure()
plt.title("Magnetometer Norm vs. Throttle Correlation")
plt.plot(df1_mags['throttle'], df1_mags['mag_norm']/avg, '*')
plt.xlabel("Throttle (norm)", weight='bold')
plt.ylabel("Magnetometer norm", weight='bold')
plt.grid()

# IMU plots
imu_fig, (ax0, ax1) = plt.subplots(2, 1, sharex=True)

ax0.set_title("IMU Sensors")
ax0.set_ylabel("Gyros (deg/sec)", weight='bold')
ax0.plot(np.rad2deg(df0_imu['p']), label='p')
ax0.plot(np.rad2deg(df0_imu['q']), label='q')
ax0.plot(np.rad2deg(df0_imu['r']), label='r')
add_regions(ax0, airborne)
add_regions(ax0, regions)
ax0.grid()
ax0.legend()

ax1.set_ylabel("Accels (m/sec^2)", weight='bold')
ax1.set_xlabel('Time (sec)', weight='bold')
ax1.plot(df0_imu['ax'], label='ax')
ax1.plot(df0_imu['ay'], label='ay')
ax1.plot(df0_imu['az'], label='az')
add_regions(ax1, airborne)
add_regions(ax1, regions)
ax1.grid()
ax1.legend()

# Attitude
att_fig, (ax0, ax1, ax2) = plt.subplots(3, 1, sharex=True)

ax0.set_title("Attitude Angles")
ax0.set_ylabel('Roll (deg)', weight='bold')
ax0.plot(np.rad2deg(df0_nav['phi']))
add_regions(ax0, airborne)
add_regions(ax0, regions)
ax0.grid()

ax1.set_ylabel('Pitch (deg)', weight='bold')
ax1.plot(np.rad2deg(df0_nav['the']))
add_regions(ax1, airborne)
add_regions(ax1, regions)
ax1.grid()

ax2.set_ylabel('Yaw (deg)', weight='bold')
ax2.plot(np.rad2deg(df0_nav['psi']))
add_regions(ax2, airborne)
add_regions(ax2, regions)
ax2.set_xlabel('Time (sec)', weight='bold')
ax2.grid()


#mpld3.show()

# Velocities
fig, (ax0, ax1, ax2) = plt.subplots(3,1, sharex=True)

ax0.set_title("NED Velocities")
ax0.set_ylabel('vn (m/s)', weight='bold')
ax0.plot(df0_gps['vn'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax0.plot(df0_nav['vn'], label='EKF')
add_regions(ax0, airborne)
add_regions(ax0, regions)
ax0.grid()

ax1.set_ylabel('ve (m/s)', weight='bold')
ax1.plot(df0_gps['ve'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax1.plot(df0_nav['ve'], label='EKF')
add_regions(ax1, airborne)
add_regions(ax1, regions)
ax1.grid()

ax2.set_ylabel('vd (m/s)', weight='bold')
ax2.plot(df0_gps['vd'], '-*', label='GPS Sensor', c='g', alpha=.5)
ax2.plot(df0_nav['vd'], label='EKF')
add_regions(ax2, airborne)
add_regions(ax2, regions)
ax2.set_xlabel('Time (secs)', weight='bold')
ax2.grid()
ax2.legend(loc=0)

if 'temp' in df0_air:
    plt.figure()
    plt.title("Air Temp")
    plt.plot(df0_air['temp'])
    plt.grid()

plt.figure()
plt.title("Airspeed (kt)")
plt.plot(df0_air['airspeed'])
plt.grid()

if 'alt_press' in df0_air:
    plt.figure()
    plt.title("Altitude (press)")
    plt.plot(df0_air['alt_press'])
    plt.grid()

if 'act' in data:
    fig = plt.figure()
    plt.title("Effectors")
    plt.plot(df0_pilot['auto_manual']+0.05, label='auto')
    if 'throttle_safety' in df0_pilot:
        plt.plot(df0_pilot['throttle_safety']+0.1, label='safety')
    plt.plot(df0_act['throttle'], label='throttle')
    plt.plot(df0_act['aileron'], label='aileron')
    plt.plot(df0_act['elevator'], label='elevator')
    plt.plot(df0_act['rudder'], label='rudder')
    plt.plot(df0_act['flaps'], label='flaps')
    add_regions(fig.gca(), airborne)
    add_regions(fig.gca(), regions)
    plt.legend()
    plt.grid()

if 'pilot' in data:
    fig = plt.figure()
    plt.title("Pilot Inputs (sbus)")
    plt.plot(df0_pilot['auto_manual']+0.05, label='auto')
    if 'throttle_safety' in df0_pilot:
        plt.plot(df0_pilot['throttle_safety']+0.1, label='safety')
    plt.plot(df0_pilot['throttle'], label='throttle')
    plt.plot(df0_pilot['aileron'], label='aileron')
    plt.plot(df0_pilot['elevator'], label='elevator')
    plt.plot(df0_pilot['rudder'], label='rudder')
    plt.plot(df0_pilot['flaps'], label='flaps')
    plt.plot(df0_pilot['aux1'], label='aux1')
    add_regions(fig.gca(), airborne)
    add_regions(fig.gca(), regions)
    plt.legend()
    plt.grid()

# Altitude
fig = plt.figure()
plt.title('Altitude')
plt.plot(df0_gps['alt'], '-*', label='GPS Sensor', c='g', alpha=.5)
plt.plot(df0_nav['alt'], label='EKF')
if 'alt_press' in df0_air:
    plt.plot(df0_air['alt_press'], label='Barometer')
add_regions(fig.gca(), airborne)
add_regions(fig.gca(), regions)
plt.ylabel('Altitude (m)', weight='bold')
plt.legend(loc=0)
plt.grid()

# Top down flight track plot
plt.figure()
plt.title('Ground track')
plt.ylabel('Latitude (degrees)', weight='bold')
plt.xlabel('Longitude (degrees)', weight='bold')
plt.plot(df0_gps['lon'], df0_gps['lat'], '*', label='GPS Sensor', c='g', alpha=.5)
plt.plot(np.rad2deg(df0_nav['lon']), np.rad2deg(df0_nav['lat']), label='EKF')
plt.grid()
plt.legend(loc=0)

# Biases
bias_fig, (ax0, ax1) = plt.subplots(2, 1, sharex=True)

# Gyro Biases
ax0.set_title("IMU Biases")
ax0.set_ylabel('Gyros (deg/s)', weight='bold')
ax0.plot(np.rad2deg(df0_nav['p_bias']), label='p bias')
ax0.plot(np.rad2deg(df0_nav['q_bias']), label='q bias')
ax0.plot(np.rad2deg(df0_nav['r_bias']), label='r bias')
add_regions(ax0, airborne)
add_regions(ax0, regions)
ax0.grid()
ax0.legend()

ax1.set_ylabel('Accels (m/s^2)', weight='bold')
ax1.plot(df0_nav['ax_bias'], label='ax bias')
ax1.plot(df0_nav['ay_bias'], label='ay bias')
ax1.plot(df0_nav['az_bias'], label='az bias')
add_regions(ax1, airborne)
add_regions(ax1, regions)
ax1.set_xlabel('Time (secs)', weight='bold')
ax1.grid()
ax1.legend()

if 'health' in data:
    # System health
    plt.figure()
    plt.title("Avionics VCC")
    if 'avionics_vcc' in df0_health:
        plt.plot(df0_health['avionics_vcc'])
    plt.plot(df0_health['main_vcc'])
    if 'load_avg' in df0_health:
        plt.plot(df0_health['load_avg'])
    plt.grid()

# Spectogram (of accelerometer normal)

L = df0_imu['time'].iloc[-1] - df0_imu['time'].iloc[0]
rate = len(df0_imu['time']) / L
M = 1024
print("Spectogram time span:", L, "rate:", rate, "window:", M)

print("Computing accel vector magnitude:")
iter = flight_interp.IterateGroup(data)
accels = []
for i in tqdm(range(iter.size())):
    record = iter.next()
    imu = record['imu']
    ax = imu['ax']
    ay = imu['ay']
    az = imu['az']
    norm = np.linalg.norm(np.array([ax, ay, az]))
    accels.append(norm)
accels = np.array(accels)

# Version 1.0

# from skimage import util
# slices = util.view_as_windows(accels, window_shape=(M,), step=100)
# #print("sample shape: ", accels.shape, "sliced sample shape:", slices.shape)
# win = np.hanning(M + 1)[:-1]
# slices = slices * win
# slices = slices.T               # for convenience
# spectrum = np.fft.fft(slices, axis=0)[:M // 2 + 1:-1]
# spectrum = np.abs(spectrum)
# #print(spectrum.shape)
# f, ax = plt.subplots()
# S = np.abs(spectrum)
# S = 20 * np.log10(S / np.max(S))
# ax.imshow(S, origin='lower', cmap='viridis',
#           extent=(df0_imu['time'].iloc[0], df0_imu['time'].iloc[-1], 0, rate / 2))
# ax.axis('tight')
# ax.set_ylabel('Frequency [Hz]')
# ax.set_xlabel('Time [s]');

# Version 2.0 -- using scipy's implementation (M used from above)
# Vertical resolution is limited by M.  To expose lower frequencies
# increase M, but this will reduce horizontal resolution.

M=1024
from scipy import signal
freqs, times, Sx = signal.spectrogram(accels, fs=rate, window='hanning',
                                      nperseg=M, noverlap=M - 100,
                                      detrend=False, scaling='spectrum')
f, ax = plt.subplots()
ax.pcolormesh(times, freqs, 10 * np.log10(Sx), cmap='viridis')
ax.set_title("Accelerometer Spectogram")
ax.set_ylabel('Frequency [Hz]')
ax.set_xlabel('Time [s]');

if False:
    # yaw
    freqs, times, Sx = signal.spectrogram(df0_imu['r'].values, fs=rate, window='hanning',
                                          nperseg=M, noverlap=M - 100,
                                          detrend=False, scaling='spectrum')
    f, ax = plt.subplots()
    ax.pcolormesh(times, freqs, 10 * np.log10(Sx), cmap='viridis')
    ax.set_title('Yaw Rate Spectogram (why?)')
    ax.set_ylabel('Frequency [Hz]')
    ax.set_xlabel('Time [s]');

if False:
    # repeat with wind heading
    wd = wind_speed.values
    freqs, times, Sx = signal.spectrogram(wd, fs=rate, window='hanning',
                                          nperseg=M, noverlap=M - 25,
                                          detrend=False, scaling='spectrum')

    f, ax = plt.subplots()
    ax.pcolormesh(times, freqs, 10 * np.log10(Sx), cmap='viridis')
    ax.set_title('Wind Direction Spectogram')
    ax.set_ylabel('Frequency [Hz]')
    ax.set_xlabel('Time [s]');

plt.show()



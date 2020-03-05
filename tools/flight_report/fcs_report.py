#!/usr/bin/python3

"""run_filter.py

Run a flight data set through a filter and output a few simple plots
Author: Curtis L. Olson, University of Minnesota
"""

import argparse
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
parser.add_argument('--flight', required=True, help='flight data log')
parser.add_argument('--wind-time', type=float, help='force a wind re-estimate with this time factor.')
args = parser.parse_args()

r2d = 180.0 / math.pi
d2r = math.pi / 180.0
m2nm = 0.0005399568034557235    # meters to nautical miles
mps2kt = 1.94384               # m/s to kts
ft2m = 0.3048
m2ft = 1.0 / ft2m

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
if 'ap' in data:
    df0_ap = pd.DataFrame(data['ap'])
    df0_ap.set_index('time', inplace=True, drop=False)
    
launch_sec = None
mission = None
land_sec = None
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
# catch a truncated flight log
if startA > 0.0:
    airborne.append([startA, air['time'], "Airborne"])

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

# Roll Attitude
fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)
ax1.set_title('Roll Angle (Positive Right)')
ax1.set_ylabel('Roll (deg)', weight='bold')
ax1.plot(df0_ap['roll'], label="Target Roll (deg)")
ax1.plot(np.rad2deg(df0_nav['phi']), label="Actual Roll (deg)")
ax1.legend()
ax1.grid()
add_regions(ax1, airborne)
add_regions(ax1, regions)
ax2.set_xlabel('Time (sec)', weight='bold')
ax2.set_ylabel('AP Aileron (norm)')
ax2.plot(df0_act['aileron'], label="AP Aileron")
ax2.legend()
ax2.grid()

# Pitch Attitude
fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)
ax1.set_title('Pitch Angle')
ax1.set_ylabel('Pitch (deg)', weight='bold')
ax1.plot(df0_ap['pitch'], label="Target Pitch (deg)")
ax1.plot(np.rad2deg(df0_nav['the']), label="Actual Pitch (deg)")
ax1.legend()
ax1.grid()
add_regions(ax1, airborne)
add_regions(ax1, regions)
ax2.set_xlabel('Time (sec)', weight='bold')
ax2.set_ylabel('AP Elevator (norm)')
ax2.plot(df0_act['elevator'], label="AP Elevator (Positive Down)")
ax2.legend()
ax2.grid()

# Yaw (body)
fig, (ax1, ax2) = plt.subplots(2,1, sharex=True)
ax1.set_title('Yaw Angle (True)')
ax1.set_ylabel('Yaw (deg)', weight='bold')
ax1.plot(df0_ap['hdg'], label="Target Yaw (deg)")
ax1.plot(np.mod(np.rad2deg(df0_nav['psi']), 360), label="Actual Yaw (deg)")
ax1.legend()
ax1.grid()
add_regions(ax1, airborne)
add_regions(ax1, regions)
ax2.set_xlabel('Time (sec)', weight='bold')
ax2.set_ylabel('AP Target Roll (deg)')
ax2.plot(df0_ap['roll'], label="AP Target Roll (Positive Right)")
ax2.legend()
ax2.grid()

# TECS Total Energy
print("Computing total energy:")
iter = flight_interp.IterateGroup(data)
tecs_totals = []
for i in tqdm(range(iter.size())):
    record = iter.next()
    if 'air' in record and 'ap' in record:
        air = record['air']
        ap = record['ap']
        if 'tecs_target_tot' in ap and 'tecs_error_total' in air:
            total = ap['tecs_target_tot'] - air['tecs_error_total']
            tecs_totals.append({ 'time': air['time'],
                                 'tecs_total': total })
    else:
        # do we care?
        pass
if len(tecs_totals):
    df1_tecs = pd.DataFrame(tecs_totals)
    df1_tecs.set_index('time', inplace=True, drop=False)

    fig, (ax1, ax2, ax3) = plt.subplots(3,1, sharex=True)
    ax1.set_title("'Total Energy' Control System")
    ax1.set_ylabel("Altitude")
    lns1 = ax1.plot(df0_ap['alt'], label="Target Alt (MSL)")
    lns2 = ax1.plot(df0_nav['alt']*m2ft, label='EKF Altitude (MSL)')
    ax1b = ax1.twinx()  # instantiate a second axes that shares the same x-axis
    ax1b._get_lines.prop_cycler = ax1._get_lines.prop_cycler
    ax1b.set_ylabel('Airspeed')
    lns3 = ax1b.plot(df0_ap['speed'], label="Target Airspeed (Kts)")
    lns4 = ax1b.plot(df0_air['airspeed'], label="Airspeed (Kts)")
    ax1b.tick_params(axis='y')
    add_regions(ax1, airborne)
    add_regions(ax1, regions)
    lns = lns1+lns2+lns3+lns4
    labs = [l.get_label() for l in lns]
    ax1.legend(lns, labs)
    ax1.grid()

    ax2.set_ylabel("Energy")
    ax2.plot(df0_ap['tecs_target_tot'], label="TECS Target Total")
    ax2.plot(df1_tecs['tecs_total'], label="TECS Total Energy")
    ax2.plot(df0_air['tecs_error_diff'], label="TECS Energy Balance Error")
    ax2.legend()
    ax2.grid()

    ax3.set_xlabel("Time (secs)", weight="bold")
    ax3.set_ylabel('AP Throttle (norm)')  # we already handled the x-label with ax1
    lns1 = ax3.plot(df0_act['throttle'], label="AP Throttle")
    lns2 = ax3.plot(df0_act['elevator'], label="AP Elevator (Positive Down)")
    ax3b = ax3.twinx()  # instantiate a second axes that shares the same x-axis
    ax3b._get_lines.prop_cycler = ax3._get_lines.prop_cycler
    ax3b.set_ylabel('Angle')
    lns3 = ax3b.plot(df0_ap['pitch'], label="AP Pitch (deg)")
    lns = lns1+lns2+lns3
    labs = [l.get_label() for l in lns]
    ax3.legend(lns, labs)
    ax3.grid()

    fig.tight_layout()  # otherwise the right y-label is slightly clipped

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

plt.show()

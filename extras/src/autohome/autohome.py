#!/usr/bin/python

from gps import *
import math
import time
import fgtelnet

host = "localhost"
port = 5402
interval = 30.0
verbose = False

report_time = time.time()
deg2rad = math.pi / 180.0

x_sum = 0.0
y_sum = 0.0
counter = 0
track = 0.0

print "Update interval is %.1f seconds" % interval

gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

try:
    while True:
        # read next available gps message
        report = gpsd.next()
	#print report
        
        # Check report class for 'DEVICE' messages from gpsd.  If
        # we're expecting messages from multiple devices we should
        # inspect the message to determine which device
        # has just become available.  But if we're just listening
        # to a single device, this may do.
        #if report['class'] == 'DEVICE':
        #    # Clean up our current connection.
        #    gpsd.close()
        #    # Tell gpsd we're ready to receive messages.
        #    gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
	#    gpsd.stream(WATCH_ENABLE|WATCH_NEWSTYLE)
	#    a = 1

        # basic sanity checks on gps position (skip report if data is bunk)
        if gpsd.fix.mode < 1:
            continue
        if math.isnan(gpsd.fix.longitude) or math.isnan(gpsd.fix.latitude):
            continue
        
        # average track (x, y components)
        if math.isnan(gpsd.fix.track) or math.isnan(gpsd.fix.speed):
            # skip track work
            a = 1
        else:
            track_rad = math.pi / 2.0 - gpsd.fix.track * deg2rad
            x = math.cos(track_rad) * gpsd.fix.speed;
            y = math.sin(track_rad) * gpsd.fix.speed;
            x_sum += x
            y_sum += y
            counter += 1
            filt_speed = math.sqrt( x*x + y*y )
            if verbose:
                print "hdg = %.1f rad = %.2f x = %.2f  y = %.2f spd = %.2f" % (gpsd.fix.track, track_rad, x, y, filt_speed)

        # send home updates at specified interval
        cur_time = time.time()
        if cur_time >= report_time + interval:
            report_time = cur_time

            if verbose:
                print
                print ' GPS reading'
                print '----------------------------------------'
                print 'latitude    ' , gpsd.fix.latitude
                print 'longitude   ' , gpsd.fix.longitude
                print 'time utc    ' , gpsd.utc,' + ', gpsd.fix.time
                print 'altitude (m)' , gpsd.fix.altitude
                print 'eps         ' , gpsd.fix.eps
                print 'epx         ' , gpsd.fix.epx
                print 'epv         ' , gpsd.fix.epv
                print 'ept         ' , gpsd.fix.ept
                print 'speed (m/s) ' , gpsd.fix.speed
                print 'climb       ' , gpsd.fix.climb
                print 'track       ' , gpsd.fix.track
                print 'mode        ' , gpsd.fix.mode
                print
                print 'sats        ' , gpsd.satellites

            # update movement average/stats
            if counter > 0:
                x = x_sum / counter
                y = y_sum / counter
            else:
                x = 0.0
                y = 0.0
            if verbose:
                print "x_sum = %.2f y_sum = %.2f counter = %d" % (x_sum, y_sum, counter)
            # zero accumulators
            x_sum = 0.0
            y_sum = 0.0
            counter = 0

            avg_speed = math.sqrt( x*x + y*y )
            if avg_speed > 0.5:
                # Only update track if speed is above a threshold of movement.
                # This way we don't get random orientations if the base station
                # isn't moving.  Note: 0.5 mps =~ 1 kt
                track = 90.0 - math.atan2( y, x ) / deg2rad
                if track < 0:
                    track += 360.0
            print "Time = %.1f avg track = %.1f x = %.2f  y = %.2f spd = %.2f" % (cur_time, track, x, y, avg_speed)

            message = "send home,%.8f,%.8f,0,%.1f" % \
                      ( gpsd.fix.longitude, gpsd.fix.latitude, track)
            print message
            try:
                t = fgtelnet.FGTelnet(host, port)
                t.send("data")
                t.send(message)
                t.quit()
            except:
                print "Cannot connect to uglink"

                    
except StopIteration:
    print "GPSD has terminated"

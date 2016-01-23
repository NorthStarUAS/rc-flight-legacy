# aura-core development summary

## Overview

Aura-core was born out of the Avior autopilot effort.  The avior is
developed by Airborne Technologies, Inc. out of Wasilla, AK.  The
original Avior flew in a small marinized (water proof) UAS and
development was funded through several NOAA.gov projects.  The Avior
development traces it's roots back to 2005-2006 and early prototypes
were flying in 2007.

The Avior system incorporated significant amount of open-source
derived code (much of it from the FlightGear open-source flight
simulator.)  Thus prior to distribution and further development, the
open-source portions of the avior were split out into their own
library and licensing was ensured to be LGPL (or more relaxed as
required) for all included modules.  Aura-core contains almost
everything needed to construct a standalone autopilot, so background
efforts have been in place through 2015 and into 2016 to fill in the
missing pieces.  As part of this, aura-core has embraced python, and
future versions will include more and more python integration, right
on board the aircraft.

## 2016 Development

* 2016-01-23: Integrate Python-based property system, paving the way
  for future python integration.

* 2016-01-18: Standardize on .cxx and .hxx for C++ source file name
  extensions

* 2016-01-15: Replace fgprops with new pyprops.  Replace C-based xml
  parser code with python xml parser.

* 2016-01-01: Work begins on replacing the original fgprops subsystem
  with a new python-based property system, exposing the entire
  property tree to native python code.

## 2015 Development

* 2015-12-14: Perform quite a bit of internal renaming for consistency
  and for the future.

* 2015-12-14: Remove the original ardupilot sensor head interface.
  This was the old/original Atmega328 based board that could drive 2
  servos accurately and 2 additional servos very coarsely.  This
  little board that did almost nothing enabled me to move beyond the
  original micronav and into a future world.

* 2015-10-06: Begin work on Goldy2 driver.  This is a Univerity of
  Minnesta UAS lab developed system that also runs on a beaglebone.
  The sensor head communicates via ethernet.

* 2015-09-16: While not flying, continuously update our ground
  altitude estimate based on current altitude.

* 2015-09-15: Remove remnants of old MNAV code/support.  Remove
  support for ancient sparkfun 6DOF imu sensor.  And a 'recalibrate'
  function fo resetting airspeed zero point.

* 2015-07-16: Add a new apm2-sensors config info message so we can log
  the apm2 serial number (for later keeping track of what flight log
  data came from which specific sensor.)

* 2015-07-15: Set system clock from gps time when gps time becomes
  valid (but only if substantially differnt from system time.)

* 2015-06-25: Make circle manager a standalone subsystem within
  aura-core.

* 2015-05-04: Log IMU temperature along with main IMU sensor data.
  EKF already estimates sensor biases (for gyros and accels).
  Developed a system to correlate EKF bias estimates with temperature
  so that each individual system can build up a temperature
  calibration fit over time as the system is flown in a variety of
  conditions.

* 2015-04-29: Add NavPy scripts to the distribution from UMN UAS lab.
  Allows rerunning all the flight data through the EKF off line and
  restimating attitude and sensor biases.  Good for offline processing
  and other types of testing and developing.

* 2015-04-22: Switch to quaternion based EKF implementation.

* 2015-02-15: Massive changes to apm2-sensor head to implement a
  'smart receiver' type system.  Mixing modes happen right on the
  aircraft (little processeor) side, not in the transmitter and not
  then replicated by the autopilot.  This allows implementing a
  universal stability/dampening system.

## 2014 (and previous)



2009-04-13: version 1.6

* Check point release prior to integrating UMN INS/GNS code and testing a 
  timer/callback architecture.

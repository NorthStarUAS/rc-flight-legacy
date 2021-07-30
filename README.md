# Rice Creek UAS high level flight controller (rc-flight)

## Welcome

rc-flight is the high level flight controller for the Rice Creek UAS
project.  It is distributed under the MIT license except where noted
(see the licenses subdirectory for details.)  rc-flight is an embedded
autopilot application for unmanned aircraft systems.

* [Watch rc-flight in action, raw and unedited](https://www.youtube.com/channel/UC_AWqZyWYvnA-h9MMcbNYyA)
* [Visit my blog for a variety of rc-flight (formerly AuraUAS) related projects and background information](http://gallinazo.flightgear.org/)

We know you have a choice in autopilots, so we appreciate you flying
with us!  The focus here is simplicity and understandability without
sacrificing performance and quality.  There is a very intentional
emphasis on writing as much of the code as possible in pure python.
There is a limited number of supported environments and sensors to
avoid complicating the architecture and build system.

rc-flight includes:

* An advanced 15-state EKF (extended kalman filter) developed by the
  University of Minnesota Aerospace Engineering Department.
  Internally the filter uses quaternion math to avoid gimbal lock
  issues.

* A sophisticated route management and waypoint following system.

* A circle hold system that holds precise circles even in strong
  winds.

* A flexible/configurable PID-based control system.  Several
  controller options are available including a classic PID controller
  and a velocity form controller, along with digital filters and
  summers.  These can be mixed and matched to create multistage
  control systems.  Gains and configurations can be tuned from the
  ground in real time.

* TECS values are computed and available as inputs to the PID
  controllers.

* Flexible device driver system.

* Property system: a system for managing and sharing hierarchically
  organized data between program modules and scripts.  The proprety
  system enables shared state (and communication) between C++ and
  python modules within a single hybrid application.

* Native python script execution and integration on board the host
  flight controll (usually a beaglebone for my projects.)

* Flexible actuator support.

* Great circle distance and heading math.

* Self learning IMU temperature and magnetometer calibration system.

* System and health monitoring

## Configure/Compiling

rc-flight is built with the python setup.py tools.  Under the hood
there are several critical C++ modules that are packaged and wrapped
for python.

1. Run: "./setup.py build" in the top level directory.

<wait a while>

2. Run: "sudo ./setup.py install"

Special note: if you find your pi or beaglebone is getting crushed
during the compile and running out of memory, consider setting up a
swap partition [temporarily] during the compile.  Instructions are in
the README-beaglebone.md file.  This can really improve your native
compile experience if you are building directly on the embedded
comptuer.  Don't forget to remove the swap partition when you are
finished building because it burns a lot of your avialable disk space.

## Development Philosophy

AuraUAS is a complete (and independent) open-source autopilot.  It
features an inexpensive DIY hardware option and a professionally built
(pick and place) hardware option.  The design philosophy is built
around creating a high quality, robust core set of features using a
standardized hardware platform.  Hardware was selected to strike a
balance between keeping the system inexpensive for a hobbyist budget,
while maintaining very high standards of performance and robustness.

This is not a project that attempts to do everything for everyone with
every possible sensor and every possible use case.  That approach adds
tremendous complexity and other projects are pursuing this approach
very effectively.

Aircraft are designed and developed with light weight as a priority in
every phase.  Analogous to that principle, AuraUAS is developed with
simplicity as a priority.  Aircraft don't always end up as light as we
would like and AuraUAS is not as simple to understand as I would like,
but know that this is a priority and something we push for at every
step.

A core element of our 'simplicity' philosophy is that the autopilot
code is divided into two main parts that run on separate processors.

1. All the sensor data collection and PWM servo control and the manual
   fail safe mode run on a Teensy-3.2 (or 3.6.)  The firmware is
   developed in the arduino enviroment and addresses all the hard
   real-time needs of the autopilot system using a simple interrupt
   service routine model.

2. All the higher level functionality runs on an embedded linux
   platform.  Currently this is a beaglebone, but the code can easily
   run on any linux-based system.  The linux code is single threaded
   and uses non-blocking I/O strategies to maintain real-time 100hz
   performance.

   As part of the push towards simpler code, the linux-side
   application is a hybrid mix of C++ and python3.  I have found that
   I can easily maintain 100hz real-time performance, even with a
   significant amount of python code in the main loop.  Opinions may
   differ, but I find that python leads to 40-50% fewer lines of code,
   and that code can be much more readable.  Embedded scripting also
   enables powerful feature development without needing to modify the
   core C++ code and recompile the firmware.

Together, the system is comprised of two simpler applications working
together as a distributed system.  You can compare this to the typical
open-source autopilot architecture that results in a giant monolithic
application which uses a complicated (and often brittle) thread-based
architecture to maintain near real-time performance.

Over the past few years AuraUAS is evolved into a mature and stable
autopilot system.  It is a rock solid work horse for many of the
University research projects I support.  Still, with any system, there
are endless feature changes and improvements to be made.

Immediate development goals include:

* recently the fmu firmware was ported from arduino to ardupilot's
  chibios-based AP_HAL to eliminate the need for custom in-house
  hardware development.  The next step is to integrate all the changes
  into an airplane and flight validate this big change.

* allow the fmu to optionally run the configurable pid controllers.

* allow the fmu to manage telemetry and logging.

* longer term my hope is to migrate all the C++ specific code directly
  onto the fmu with enough basic functionaly to support common tasks.
  The rc-flight code then will act more like a host (or companion)
  computer for managing higher level mission tasks or doing
  experimental tasks "offboard".

* [done] Support for structured hdf5 data log export.

* [done] Python 3 port.

* [done] Implement a hand-thrown auto-launch task.  This is written
  as a python task (within the python-based mission system) and
  produces very very solid and stable launch results for hand thrown
  airplanes (such as a Skywalker or Talon.)

* [done] Add support for computing survey routes on board the aircraft.  (The
  issue here is the brittleness and time required to send 100's of
  waypoints up to an aircraft over a radio modem link.  Instead we can
  just send the area to be covered and some camera parameters and the
  aircraft can compute it's route internally.)

* [done!] Redesign and modernize the inexpensive reference hardware. 

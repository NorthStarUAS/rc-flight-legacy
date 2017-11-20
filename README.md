# aura-core

## Welcome

Aura-core is the heart of the AuraUAS project.  It is distributed
under the LGPL license except where noted (see the licenses
subdirectory for details.)  AuraUAS is an embedded autopilot
application for unmanned aircraft systems.

Watch AuraUAS in action, raw and unedited:

    https://www.youtube.com/channel/UC_AWqZyWYvnA-h9MMcbNYyA

Please visit my blog for a variety of AuraUAS related projects and
background information:

    http://gallinazo.flightgear.org/

Aura-core includes:

* An advanced 15-state EKF (extended kalman filter) developed by the
  University of Minnesota Aerospace Engineering Department.
  Internally the filter uses quaternion math to avoid gimbal lock
  issues.

* A sophisticated route management and waypoint following system.

* A sophisticated circle hold system that holds precise circles even
  in strong winds.

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
  organized data between program modules and scripts.

* Native python script execution and integration on board.

* Flexible actuator support.

* Great circle distance and heading math.

* Self learning IMU temperature calibration system.

* System and health monitoring


## Configure/Compiling

Aura-core uses the automake/autoconf tool set and is built like many
popular open-source projects.

1. Run: "./autogen.sh" in the top source directory.

2. Make a subdirectory called build (you can have multiple build
   directories setup with different build options, for example,
   supporting different architectures, debug vs. optimized release
   builds, profiling builds, etc.)

3. Run "../configure CFLAGS="-Wall -O3" CXXFLAGS="-Wall -O3"

4. Run "make"


## Development Road Map

Recently aura-core became a complete standalone open-source autopilot.
Development added a main() loop and an integrated python mission
manager that is extensible with your own python code.  Python is
running right on board the aircraft and playing a key functional role
in the operation of the autopilot.  The python integration has been
tested in simulation and now in flight with excellent results.  Deeply
integrating python into the core autopilot was a major strategic
decision and perhaps a major distinguishing factor compared to many
other autopilot ecosystems that currently exist and are being
developed.

Immediate development goals include:

* Add support for computing survey routes on board the aircraft.  (The
  issue here is the brittleness and time required to send 100's of
  waypoints up to an aircraft over a radio modem link.  Instead we can
  just send the area to be covered and some camera parameters and the
  aircraft can compute it's route internally.)

* Redesign and modernize the inexpensive reference hardware. 

Longer term goals include:

* Streamline and expand the surveying capabilities of this system.

* Easier support for researchers and DIY'ers to add their own python
  tasks and code deeply integrated into the main loop.

* Modernization of the ground station interface (the visual
  interactive portion.)
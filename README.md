# aura-core

## Welcome

Aura-core is the heart of the AuraUAS project.  It is distrubted under
the LGPL license except where noted (see the licenses subdirectory for
details.)  AuraUAS is an advance embedded application for unmanned
aircraft systems.  Aura-core includes:

* An advanced 15-state EKF (extended kalman filter) developed by the
  University of Minnesota Aerospace Engineering Department.
  Internally the filter uses quaternion math to avoid gimbal lock
  issues.

* A sophisticated route management and following system.

* A sophisticated circle hold system that holds precise circles even
  in strong winds.

* A flexible/configurable PID-based control system.  The primary PID
  controller is provided in the 'velocity' form.  Gains and
  configurations can be tuned from the ground in real time.

* Flexible device driver system.

* Property system: a system for managing and sharing hierarchically
  organized data between program modules and scripts.

* Native python script execution and integration on board.

* Flexible actuator support.

* Great circle and wgs84 distance and heading math.

* Temperature/IMU calibration system.

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

## Future work

Currently (at the time of writing) aura-core is just a set of
libraries that can be stiched together from your own main() to produce
a working autopilot.  The longer term goal is to provide a suitable
main() here along with a Python based mission/task managment system so
that aura-core is a complete, standalone, open-source autopilot system
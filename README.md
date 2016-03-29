# aura-core

## Welcome

Aura-core is the heart of the AuraUAS project.  It is distributed under
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
manager that is extensible with your own python code.  (Python is
running right on board aircraft and playing a key functional role in
the operation of the autopilot.)  The python integration has been
tested in simulation and now in flight with excellent results.  Deeply
integrating python into the core autopilot was a major strategic
decision and perhaps a major distinguishing factor compared to many
other autopilot ecosystems that currently exist and are being
developed.

Immediate development goals include:

* rewriting the ground station link code in python, jettisoning a
  significant amount of historic cruft, massively improving the
  performance, as well as improving websocket compatibility with a
  much wider variety of systems (i.e. adding iOS support.)

* rewriting the sensor interface and then subsequently leveraging that
  work to improve the determinism and reducing latency in the main
  loop.

Longer term goals include:

* easier support for researchers and DIY'ers to add their own python
  tasks and code deeply integrated into the main loop.

* mavlink support.

* more python.  Briefly: I like python, I like compact source code and
  efficient representation of logic and ideas.  I equate code
  efficiency with code that is easier to read and understand, as well
  as less likely to hide obscure bugs.  I like that many people find
  it easier to write python than C/C++.  I like the amazing breadth
  and depth of it's supporting libraries.  I believe wise use of
  python can bring substantial value to an open-source autopilot.  I
  understand some people might be skeptical about such deep python
  integration into the main autopilot system, and if you are one of
  those people, I welcome you to Minnesota for a flight demo.

* Modernization of the ground station interface (the visual
  interactive portion.)
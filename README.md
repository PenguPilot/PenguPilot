![PenguPilot](https://raw.github.com/PenguPilot/PenguPilot/master/doc/PenguPilot.png)

[![Build Status](https://travis-ci.org/PenguPilot/PenguPilot.svg?branch=master)](https://travis-ci.org/PenguPilot/PenguPilot)

What is PenguPilot?
===================
**PenguPilot is a GNU/Linux based Multi-Rotor UAV Autopilot.**

Unlike other platforms, PenguPilot does not require a dedicated low-level microcontroller for real-time

   * sensor data acquisition,
   * data fusion / filtering, and
   * motor control.

Instead, the whole flight infrastructure is based on user-space tasks in a Linux operating system (PREEMPT or PREEMPT_RT). This means, if the control process is killed or the kernel panics, your UAV is killed as well :-) **Already scared**? If not, please read on...

**Some highlights of the PenguPilot software are**:

- component based software architecture
- memory protection among critical and non-critical components (tasks)
- message-passing among components via ZeroMQ/MessagePack/Protobuf
- process/priority management and dependency tracking
- online parameter configuration usable for in-flight parameter updates
- SSH based user interfaces for parameters, calibration, ...
- blackbox functionality to log sensor values at each control step to sdcard
- sensor data replay capabilitiy for offline PC-based optimization, filter tuning, ...
- quick deployment to new hardware possible (Linux HAL)


**Currently, PenguPilot supports the following computer-on-modues**:

- Gumstix Overo (custom Gentoo Linux)
- Raspberry Pi (Respbian)
- Odroid U3 (custom Gentoo Linux)

What's different compared to other Approaches?
---------------------------------------------------------------------

The three main factors that make PenguPilot based systems different, compared to microcontroller-based autopilots, are the

  * **Linux operating system**, the
  * powerful underlying **computer-on-module** (COM), and
  * PenguPilot's **component-based software architecture**.

These factors generate a lot of possibilities and benefits for UAV software development, summarized as follows:

* **Powerful Hardware and OS**:
   - no microcontroller-like flash/RAM memory restrictions
   - high memory bandwidth allows to log every sensor value per control step
   - memory protection for improved task safety
   - USB host support (Cameras, WiFi and UMTS sticks, FTDI's, ...)
   - symmetric multi-processing (e.g. ODROID U3)

* **Advanced Programming Possibilities**:
  - high-level programming languages and libraries (Bash, Python/numpy)
  - rich networking and routing functionality
  - file system abstractions instead of raw memory access
  - debugging via GDB and code profiling without extra effort
  - myriads of open source device drivers and software libraries

* **Advanced Field Software Development**:
  - software development on the UAV via SSH from virtually any device
  - native source code compilation (no cross-compiler required)
  - on-device software version control via Git


Filesystem Contents
===============

Flight Infrastructure:

- [autopilot](autopilot): real-time control running at 200Hz
- [icarus](icarus): high-level flight management service
- [blackbox](blackbox): receives logging data containing every sensor input of the autopilot to sd card
- [powerman](powerman): power management and monitoring service; warns the user if the battery is low
- [gpsp](gpsp): gps publisher, similar to gpsd but much simpler; uses [NMEALib](https://github.com/AHR-Project/nmealib)
- [remote](remote): remote control data parser and publisher
- [geomag](geomag): reads gps position and date; publishes magnetic declination in degrees
- [ads1x15](ads1x15): (I2C ADC driver for Raspberry PI)
- [arduino](arduino): (ODROID U3)
- [twl4030_madc](twl4030_madc): (Gumstix Overo)

Supporting Infrastructures:

- [config](config): configuration files
- [scl](scl): signaling and communication link (IPC framework), see [config/system.yaml](config/system.yaml)
- [opcd](opcd): online parameter configuration daemon, see [config/params.yaml](config/params.yaml)
- [svctrl](svctrl): service management and control utility, see [config/services.yaml](config/services.yaml)
- [shared](shared): shared Libraries for threading and other common tasks

Additional Features:

- [aircomm](aircomm): encrypted aerial communication daemon, using NRF24L01+
- [display](display): shows battery, mem, cpu, satellite status via I2C SSD1307 128x64 display from Adafruit
- [wifi_sensor](wifi_sensor): publishes wireless network data acquired via iwlist
- [wifi_loc](wifi_loc): combines gps measurements and wifi scan results and publishes it

Build System and Environment:

- [site_scons](site_scons): related to build system
- [SConstruct](SConstruct): scons build file
- [scripts](scripts): various scripts, e.g. bashrc

Software Dependencies:
- Gentoo: app-admin/sudo app-misc/screen dev-lang/python dev-lang/swig dev-libs/glib dev-libs/libyaml dev-libs/msgpack dev-libs/protobuf dev-libs/protobuf-c dev-python/imaging dev-python/msgpack dev-python/numpy dev-python/psutil dev-python/python-daemon dev-python/pyyaml dev-python/pyzmq dev-util/scons sys-power/cpufrequtils dev-python/pyproj sci-libs/gdal
- Ubuntu: realpath scons swig protobuf-compiler python-protobuf libmsgpack-dev libprotobuf-dev python-yaml protobuf-c-compiler libprotobuf-c0-dev libzmq-dev python-zmq libyaml-dev libglib2.0-dev python-daemon python-termcolor libpython-dev


Service Dependencies
--------------------
The service dependencies of PenguPilot are depicted in the figure below:

![Services Dependency Graph](https://raw.github.com/PenguPilot/PenguPilot/master/doc/ServicesGraph.png)

Services indicated in yellow are platform-specific, as specified in file config/services.yaml.



Example UAV System
------------------

Here's an example of a Gumstix Overo Air based PenguCopter with GPS receiver and I2C OLED display:

![PenguCopter](https://raw.github.com/PenguPilot/PenguPilot/master/doc/GumstixCopter.jpg)

[PenguCopter Video](http://vimeo.com/98649107)


[Flying Penguins from BBC :)](https://www.youtube.com/watch?v=9dfWzp7rYR4)


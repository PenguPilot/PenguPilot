
![PenguPilot](https://raw.github.com/PenguPilot/PenguPilot/master/doc/PenguPilot.png)


[![Build Status](https://travis-ci.org/PenguPilot/PenguPilot.svg?branch=master)](https://travis-ci.org/PenguPilot/PenguPilot)

Overview
========
PenguPilot is a Free and Open Source Multi-Rotor UAV autopilot for Linux-capable computer modules. Currently, the following platforms are supported:
- Gumstix Overo (custom Gentoo Linux)
- Raspberry Pi (Respbian)
- Odroid U3 (custom Gentoo Linux)

In contrast to other platforms, PenguPilot does not require a dedicated low-level microcontroller for real-time sensor data acquisition and motor control. The whole control process is implemented in a high-priority user-space task, which is ideal for prototyping and experimentation.

What's different compared to other Autopilots?
----------------------------------------------

The two main factors that make PenguPilot different are the
  * **Linux operating system**, and the
  * powerful underlying **computer-on-module** (COM).

These factors generate a lot of possibilities and benefits, and influence the way how software development is conducted on a UAV:

* **Advanced programming and rich system interfaces**:
  - no microcontroller-like flash/RAM memory restrictions
  - high-level programming languages and libraries (Bash, Python/numpy)
  - flexible networking functionality (WLAN, UMTS)
  - file system abstractions instead of raw memory access
  - debugging via GDB and code profiling without extra effort
  - high memory bandwidth allows to log every sensor value per control step

* **Memory protection among software components**:
  - high-priority, safety critical components (control and flight management)
  - low-priority, non safety critical components (parameter management, communication)
  - coexistence of open source and closed source software

* **Advanced in-field software development and management**:
  - software development on the UAV via SSH from virtually any device
  - native source code compilation (no cross-compiler required)
  - computation-intensive compilation via QEMU (e.g. Linux updates)
  - on-device software version control via Git

To make full use of these benefits, it is important to choose the right tools for development.
Thus, PenguPilot's architecture allows to distribute control code among several processes (e.g. high-level control in Python and low-level control in C). Currently, components are implemented in C and Python, communicating efficiently via ZeroMQ and Protobuf/MessagePack.


Filesystem Contents
-------------------

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

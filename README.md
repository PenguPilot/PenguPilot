     ___________________________________________________
    |  _____                       _____ _ _       _    |
    | |  __ \                     |  __ (_) |     | |   |
    | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
    | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
    | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
    | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
    |                   __/ |                           |
    |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
    |___________________________________________________|

[![Build Status](https://travis-ci.org/PenguPilot/PenguPilot.svg?branch=master)](https://travis-ci.org/PenguPilot/PenguPilot)

Overview
--------
PenguPilot is a Free and Open Source Multi-Rotor UAV autopilot for Linux-capable computer modules like the Gumstix Overo, Raspberry Pi or the Odroid-U3.
Sensor data acquisition, state estimation and feed-back control runs as a high-priority Linux user-space task, which is ideal for prototyping and experimentation.
PenguPilot's architecture allows to distribute the control code among several processes (e.g. high-level control in Python and low-level control in C).
Currently, components are implemented in C and Python, communicating efficiently via ZeroMQ and Protobuf/MessagePack.

Contents
--------

Flight Infrastructure:
- __autopilot__: real-time control running at 200Hz
- __hlfm__: high-level flight management service
- __blackbox__: receives logging data containing every sensor input of the autopilot to sd card
- __powerman__: power management and monitoring service; warns the user if the battery is low
- __gpsp__: gps publisher, similar to gpsd but much simpler; uses [NMEALib](https://github.com/AHR-Project/nmealib)
- __remote__: remote control data parser and publisher
- __geomag__: reads gps position and date; publishes magnetic declination in degrees
- __ads1x15__: (Raspberry PI)
- __arduino__: (ODROID U3)
- __twl4030_madc__: (Gumstix Overo)
- __config__: configuration file(s)


Supporting Infrastructures:

- __config__: configuration files
- __scl__: signaling and communication link (IPC framework), see [config/system.yaml](config/system.yaml)
- __opcd__: online parameter configuration daemon, see [config/params.yaml](config/params.yaml)
- __svctrl__: service management and control utility, see [config/services.yaml](config/services.yaml)
- __shared__: shared Libraries for threading and other common tasks

Additional Features:

- __aircomm__: encrypted aerial communication daemon, using NRF24L01+
- __display__: shows battery, mem, cpu, satellite status via I2C SSD1307 128x64 display from Adafruit
- __wifi_sensor__: publishes wireless network data acquired via iwlist
- __wifi_loc__: combines gps measurements and wifi scan results and publishes it

Build System and Environment:

- __site_scons__: related to build system
- __SConstruct__: scons build file
- __scripts__: various scripts, e.g. bashrc

Software Dependencies:
- Gentoo: app-admin/sudo app-misc/screen dev-lang/python dev-lang/swig dev-libs/glib dev-libs/libyaml dev-libs/msgpack dev-libs/protobuf dev-libs/protobuf-c dev-python/imaging dev-python/msgpack dev-python/numpy dev-python/psutil dev-python/python-daemon dev-python/pyyaml dev-python/pyzmq dev-util/scons sys-power/cpufrequtils
- Ubuntu: realpath scons swig protobuf-compiler python-protobuf libmsgpack-dev libprotobuf-dev python-yaml protobuf-c-compiler libprotobuf-c0-dev libzmq-dev python-zmq libyaml-dev libglib2.0-dev python-daemon python-termcolor

Example UAV System
------------------

Here's an example of a Gumstix Overo Air based PenguCopter with GPS receiver and I2C OLED display:

![PenguCopter](https://raw.github.com/PenguPilot/PenguPilot/master/doc/GumstixCopter.jpg)

[Flying Penguins from BBC :)](https://www.youtube.com/watch?v=9dfWzp7rYR4)


Service Dependencies
--------------------
The service dependencies of PenguPilot are depicted in the figure below:

![Services Dependency Graph](https://raw.github.com/PenguPilot/PenguPilot/master/doc/ServicesGraph.png)

Services indicated in yellow are platform-specific, as specified in file config/services.yaml.

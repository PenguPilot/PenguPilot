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

Example UAV System
------------------

Here's an example of a Gumstix Overo Air based PenguCopter with GPS receiver and I2C OLED display:

![PenguCopter](https://raw.github.com/PenguPilot/PenguPilot/master/doc/GumstixCopter.jpg)

![PenguCopter](http://vimeo.com/98649107)



[Flying Penguins from BBC :)](https://www.youtube.com/watch?v=9dfWzp7rYR4)


Service Dependencies
--------------------
The service dependencies of PenguPilot are depicted in the figure below:

![Services Dependency Graph](https://raw.github.com/PenguPilot/PenguPilot/master/doc/ServicesGraph.png)

Services indicated in yellow are platform-specific, as specified in file config/services.yaml.

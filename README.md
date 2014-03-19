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


Overview
--------
PenguPilot is a Free and Open Source Multi-Rotor UAV autopilot for Linux-capable computer modules like the Gumstix Overo, Raspberry Pi or the Odroid-U3.
Sensor data acquisition, state estimation and feed-back control runs as a high-priority Linux user-space task, which is ideal for prototyping and experimentation.
PenguPilot's architecture allows to distribute the control code among several processes (e.g. high-level control in Python and low-level control in C).
Currently, components are implemented in C and Python, communicating efficiently via ZeroMQ and Protobuf/MessagePack.

Contents
--------
Flight Infrastructure:
- autopilot:
  - service: the autopilot service
  - tools: calibration and other useful utilities
- blackbox: black box service for logging every sensor input of the autopilot to harddisk/sdcard
- powerman: power management and monitoring service
- gps: gps service, similar to gpsd but more efficient and much simpler using nmeablib
- icarus: high-level control service for complex commands

Supporting Infrastructures:
- scl: signaling and communication link (IPC framework), see config/system.yaml
- opcd: online parameter configuration daemon, see config/params.yaml
- svctrl: service management and control utility, see config/services.yaml
- shared: shared Libraries for threading and other common tasks
- config: configuration files
- setup: config file creation scripts (EXPERIMENTAL)

Additional Features:
- aircomm: encrypted aerial communication daemon, using NRF24L01+
- display: shows battery, mem, cpu, sattelite status via I2C SSD1307 128x64 display from Adafruit
- wifi_sensor: published wireless network data acquired via iwlist
- wifi_loc: combines gps measurements and wifi scan results and publishes it

Build System and Environment:
- site_scons: related to build system
- SConstruct: scons build file
- scripts: various scripts, e.g. bashrc

Library dependencies:
- C: msgpack, meschach, protobuf-c, yaml, zeromq, glib
- Python: psutil, pyyaml, protobuf, msgpack, pyzmq, swig, python-daemon, numpy
- System recommended: sudo, git, i2c-tools, screen...

Example UAV System
------------------

Here's an example of a Gumstix Overo Air based PenguCopter with GPS and display:

![PenguCopter](https://raw.github.com/PenguPilot/PenguPilot/master/doc/GumstixCopter.jpg)

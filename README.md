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
PenguPilot is a Free and Open Source Multi-Rotor UAV autopilot
for Linux-based computer modules.
The whole state estimation and control system runs as a Linux
user-space task, which is ideal for prototyping and experimentation.
PenguPilot's architecture allows to distribute the control code
among several processes (e.g. high-level control and low-level control).

Contents
--------

Build System and Environment:
- site\_scons: related to build system
- SConstruct: scons build file
- scripts: Various Scripts

Supporting Infrastructures:
- setup: config file creation scripts
- scl: Signaling and Communication Link
- opcd: Online Parameter Configuration Daemon
- svctrl: Service Management and Control Utility
- shared: Shared Libraries

Flight Infrastructure:
- autopilot: autopilot software
- powerman: Power Management Daemon
- sensors: Sensor Publisher Services
- gps: gps sensor, similar to gpsd
- icarus: high-level control and complex commands

![Build Status](https://travis-ci.org/PenguPilot/PenguPilot.png)

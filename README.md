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
Flight Infrastructure:
- autopilot: autopilot service and calibration tools
- blackbox: black box service and tools
- powerman: power management and monitoring service
- gps: gps service, similar to gpsd but more efficient and much simpler
- icarus: high-level control service for complex commands

Supporting Infrastructures:
- scl: signaling and communication link (IPC framework)
- opcd: online parameter configuration daemon
- svctrl: service management and control utility
- shared: shared Libraries
- config: system architecture and parameter config files
- setup: config file creation scripts (EXPERIMENTAL)
- remote: remote control reader service (EXPERIMENTAL)

Build System and Environment:
- site_scons: related to build system
- SConstruct: scons build file
- scripts: various scripts, e.g. bashrc for sourcing


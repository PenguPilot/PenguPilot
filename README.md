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


PenguPilot Contents
-------------------

Build System and Supporting Infrastructures:

- scripts: Various Scripts
- SConstruct: Build System
- site\_scons: related to build system
- scl: Signaling and Communication Link
- opcd: Online Parameter Configuration Daemon
- shared: shared libraries

Flight Infrastructure:
- autopilot: autopilot software
- config: configuration files (opcd, scl, svctrl)
- powerman: Power Management Daemon
- sensors: Sensor Publisher Services


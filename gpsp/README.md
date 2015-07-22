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


GPS Publisher
=============

The gps publisher reads data from a gps device and publishes
this data in order to make it available to other services like the autopilot.

Starting the Service
--------------------

    $ pp_svctrl --start gpsp

Interfacing
-----------

The service publishes data on two sockets:
   - gps: provides gps fix information
   - sats: provides satellite information

Additional Information
----------------------

GPSP uses a third-party library, which is shipped with PenguPilot:
[NMEALib](https://github.com/AHR-Project/nmealib)


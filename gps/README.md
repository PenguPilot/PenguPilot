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


GPS Subsystem
=============

The gps service reads data from a gps device and publishes
this data in order to make it available to other services like the autopilot.

* The data format used for serialization is defined in the protobuf message
format located in the "shared" folder.
* as an additional feature, the gps service sets the system's date and time based on GPS time and position information (timezone).
* the service uses a third-party library, which is shipped with PenguPilot:
[NMEALib](https://github.com/AHR-Project/nmealib)


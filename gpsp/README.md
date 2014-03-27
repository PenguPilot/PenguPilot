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

    $ pp_svctrl --start gps
    dependency resolution order: ['opcd', 'gps']
    starting opcd ... ~/PenguPilot/opcd/service/opcd.py [OK]
    starting gpsp ... ~/PenguPilot/gpsp/service/gpsp [OK]

As you can see, the service depends on the opcd, which provides configuration parameters such as the serial port, and the serial baud rate.

Interfacing
-----------

To interface with the gps publisher, please have a look at the config/system.yaml. There are already some services using the gps data, such as the autopilot and the display.

The data format used for serialization is defined in the protobuf message
format located in the "shared" folder.

Additional Information
----------------------

As an additional feature, the gps publisher sets the system's date and time based on GPS time and position information (timezone).
GPSP uses a third-party library, which is shipped with PenguPilot:
[NMEALib](https://github.com/AHR-Project/nmealib)


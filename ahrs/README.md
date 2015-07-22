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


Attitude and Heading Reference System
=====================================

- [service/ahrs](ahrs):
  - subscribes to sockets "gyro", "acc", "mag", and "decl" (magnetic declination)
  - uses Quaternion implementation of the [Mahony DCM](https://gentlenav.googlecode.com/files/MahonyPapers.zip)
  - publishes Euler Angles as an array (yaw, pitch, roll) on socket "orientation"

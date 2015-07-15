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


Current Magnetometer Compensation
=================================

This service compensates motor current effects on the magnetometer using linear regression.
Assume that there is a linear function f, which works as follows:

mag_compensated = f(mag_raw, current)

In order to determine function f, the MAV needs to be mounted stationary,
while its moters are activated and driven between minimum and maximum thrust.
During this sequence, current and magnetometer readings are collected and stored.
Using numpy least-squares fitting, offset and scale coefficients are computed.

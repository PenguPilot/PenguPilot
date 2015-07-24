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


ODROID U3 IO Shield
===================

This service interfaces to the [U3 IO Shield](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G138760240354) via "/dev/ttySAC0" and allows to receive ADC data (voltage, current) and remote control channels.

File Contents:

- [checksum.c](checksum.c): checksum implementation
- [checksum.h](checksum.h): checksum interface
- [main.c](main.c): remote control and ADC bridge service
- [power_common.h](power_common.h): power protocol common interface
- [power_parse.c](power_parse.c): power parser implementation
- [power_parse.h](power_parse.h): power parser interface
- [ppm_common.h](ppm_common.h): PPM common interface
- [ppm_parse.c](ppm_parse.c): PPM parser implementation
- [ppm_parse.h](ppm_parse.h): PPM parser interface

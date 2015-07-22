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


Accelerometer Calibration Subsystem
===================================

- [service/acc_cal.py](service/acc_cal.py):
  - subscribes to socket "acc\_raw"
  - publishes on socket "acc"
- [tools/acc_cal.py](tools/acc_cal.py):
  - computes calibration coefficients after rotating the device in arbitrary directions
  - stores coefficients via [opcd](../opcd), section acc_cal:
    - acc_bias_x
    - acc_bias_y
    - acc_bias_z
    - acc_scale_x
    - acc_scale_y
    - acc_scale_z
- [tools/acc_magnitude.py](tools/acc_magnitude.py):
  - prints acc vector magnitude to verify calibration - this should always print values close to 9.81 (1G)

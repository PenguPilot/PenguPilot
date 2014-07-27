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


Internal sensor code:

* [itg3200](itg3200): gyroscope
* [mpu6050](mpu6050): gyroscope and accelerometer
* [bma180](bma180): accelerometer
* [hmc5883](hmc5883): magnetometer
* [ak8975c](ak8975c): magnetometer
* [ms5611](ms5611): barometric pressure sensor
* [i2cxl](i2cxl): ultrasonic sensor

External sensor code:

* [scl_power](scl_power): voltage/current
* [scl_rc](scl_rc): remote control channels
* [scl_gps](scl_gps): gps latitude/longitude/altitude/ground speed
* [scl_mag_decl](scl_mag_decl): magnetic declination
* [scl_elevmap](scl_elevmap): SRTM3 elevation at current GPS position

Additional utility code:
* [util](util)


Sensor reading code is integrated in the autopilot (internal), if:

* the sensor has to be read very quick (logic or)
* reading the sensor is not helpful outside of the autopilot

As an example, the voltage information is relevant for system health
and battery safety, even if the autopilot service is not running.


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


Autopilot
=========

Purpose
-------

This service implements the real-time low-level flight controller.


Filesystem Contents
-------------------

* [main_loop](main_loop): main control flow
* [sensors](sensors): sensor drivers
* [filters](filters): filter code
* [actuators](actuators): motors
* [platform](platform): COM / sensor / actuator compositions
* [control](control): speed and position controllers
* [force_opt](force_opt): force optimizations
* [estimators](estimators): orientation and position filters
* [flight_logic](flight_logic): manual and auto flight logic
* [state](state): (logical) system state estimation
* [interface](interface): command interface
* [blackbox](blackbox): blackbox data publisher
* [util](util): utility code


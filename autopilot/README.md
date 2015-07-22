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


Python Autopilot Implementation
===============================

- [service/autopilot.py](service/autopilot.py):
  - service used to provide basic autonomous flight capabilities, including basic commands such as:
    - takeoff
    - land
    - move
    - stop
    - rotate
  - implements a flight state machine in [service/flightsm.py](service/flightsm.py)
  - subscribes to sockets "orientation", "pos_speed_est_neu"
  - publishes state machine state updates to socket "ap_state"
  - uses the "[Control System API](../api/ctrl_api.py)" to interact with the controller services

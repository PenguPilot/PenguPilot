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


OPCD Subsystem
--------------
The PenguPilot Online Parameter Configuration Daemon (OPCD) 
service is located in the folder "service".
OPCD reads a parameters configuration file and provides
these parameters using a request-reply and using a publish interface.
Folder "shell" provides an interactive Python-based command shell
for retrieving/setting configuration parameters.
The bindings for C and Python are located in the "bindings" folder.

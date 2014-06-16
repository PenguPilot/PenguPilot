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

The Online Parameter Configuration Service (OPCD) is a low-priority service managing various configuration parameters stored in a YAML based configuration format.
The configuration consists of two files, a standard file and a override file, in which parameter updates to the standard file are written.
The override file is only valid if the structure matches the standard file.
Whenever new configuration parameters need to be added, these first have to be added to the standard file.
Three types of parameters are currently supported:

* __floats__ (controller gains, filter coefficients, ...),

* __integers__ (bus id's, debug levels, ...), 

* __strings__ (device paths, ...)

OPCD allows to request parameters using a request-reply interface and publishes parameter
updates via a publish socket in order to inform subscribers about parameter updates.
The request interface is also used by a command-line shell named __pp_opcd_shell___.
This shell is used for retrieving and updating configuration parameters. Main functions of this shell are get(key_or_regex), set(key, value) and persist() for saving the parameters.
Here, regex stands for a regular expression.
The shell is GNU readline compatible and features a history stored among invocations.


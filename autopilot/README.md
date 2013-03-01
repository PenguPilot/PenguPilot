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


Autopilot Subsystem
===================
The autopilot service is located in the folder "service",
along with a debug logger service.
Folder "shell" provides an interactive Python-based command shell
for controlling the autopilot manually.
The logging framework located in "logging" consists of a
general "text-based" status logger and a real-time
binary "debugging" logger.


PenguPilot Logging Framework
============================

The PenguPilot logging framework is intended to enable various applications
by logging each input variable of the control loop in each time step.

Applications
------------

   * continuous integration and software tests
   * rare event logging (e.g. crash, sensor failures)
   * offline filter tuning
   * performance evaluation

Approach
--------
To realize a logging framework capable of writing real-time data to
flash memory, synchronous write access is not feasible,
as interrupting the real-time processing easily leads to flight instabilities.

Thus, we split the logging framework into real-time and a non-real-time
parts, which are linked by buffered IPC, as follows:

(**AutoPilot**) ----[Buffer]---> (**DebugWriter**)

Moving the log write process to another address space also increases the system's stability:
Exceptions caused by the flash drive or by file system errors might only cause
a crash in the log writer, but do not influence the auto pilot.

Data Format
-----------
The requirements for the logging data format can be summarized as follows:

   * low CPU load
   * low flash memory usage (binary format)
   * streaming capability
   * C and Python support
   * support for string, integer, float

We choose [MessagePack](http://www.msgpack.org) as the serialization format.
A log file consists of a single header consisting of n strings,
and m measurement arrays which must have length n also.
The data arrays must contain only integer or float values.
The principle file layout is depicted below:

           c_1:        c2:        c3:
    r_0: "gyro_x" | "gyro_y" | "gyro_z" [header]
    r_1:  0.0013  |  1.0210  |  2.0106  [log data]
    r_2:  0.0305  |  1.1922  |  2.5023  [log data]
    r_3:  0.0707  |  1.3241  |  2.4321  [log data]

Files and Tools
---------------
The debug writer service is started/stopped via "svctrl --start|--stop debug_writer".

It stores the log files in PenguPilot's log folder, which is typically "$HOME/.PenguPilot/log".
In this folder, "core_debug.msgpack" is a symbolic link to the most recent log file.
A new log file with creation date/time in its name is created when the service is started.

**CAUTION**: If the *autopilot is restarted without restarting the debug writer*,
an additional header will appear in the log file.



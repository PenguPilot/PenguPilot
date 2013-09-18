PenguPilot BlackBox
===================

The PenguPilot BlackBox is intended to enable various applications by logging various variables of the autopilot's control loop in each step, such as:
   * continuous integration and software tests
   * rare event logging (e.g. crash, sensor failures)
   * offline filter tuning
   * performance evaluation

Files and Tools
---------------
The service is started/stopped using the following command:

   $ pp\_svctrl --start|--stop blackbox

It stores the log files in PenguPilot's log folder, which is typically "$HOME/.PenguPilot/log". In this folder, "core\_debug.msgpack" is a symbolic link to the most recent log file.
A new log file with creation date/time in its name is created when the service is started. **CAUTION**: If the *AutoPilot is restarted without restarting the  BlackBox*,
an additional header will appear in the log file. This might lead to confusions.

Approach
--------
To realize a logging framework capable of writing real-time data to flash memory, synchronous write access is not feasible,
as interrupting the real-time processing easily leads to flight instabilities. Thus, the logging framework is split into a real-time and a non-real-time
part, which are linked by buffered IPC, as follows:

(**AutoPilot**) ----[Buffer]---> (**BlackBox**)
Address Space A                  Address Space B

Moving the log write process to another address space also increases the system's stability:
Exceptions caused by accessing flash memory might only cause a crash in the BlackBox,
but do not influence the AutoPilot.

Data Format
-----------
The requirements for the logging data format are summarized as follows:
   * low CPU load
   * low flash memory usage (binary format)
   * streaming capability
   * C and Python support
   * support for string, integer, float
Based on a survey on serialization formats covering these desirable properties,
[MessagePack](http://www.msgpack.org) is selected.

A log file consists of a single header consisting of n strings,
and m measurement vectors of length n.
The data vectors must consist only of integer or float values.

The figure below depicts the file layout:

           c_1:        c2:        c3:
    r_0: "gyro_x" | "gyro_y" | "gyro_z" [header]
    r_1:  0.0013  |  1.0210  |  2.0106  [log data]
    r_2:  0.0305  |  1.1922  |  2.5023  [log data]
    r_3:  0.0707  |  1.3241  |  2.4321  [log data]


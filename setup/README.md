
PenguPilot Config File Setup
============================

The approach of this software is to configure the system using the "Python" programming language.
This allows to perform more complex pre-computations compared to what would be possible in
a configuration file definition language such as YAML.
Thus, the approach allows to decouple computations conducted in the in-flight components to
pre-flight components, reducing the probability of system malfunctions e.g. due to invalid memory access.

The tool generates various local configuration files, such as:
   * services.yaml for svctrl
   * priorities.yaml for process priority configuration
   * system.yaml for defining inter-process communication
   * params-base.yaml for configuration via opcd

It allows to define selectable profiles for various orthogonal aspects like:
   * platform configuration (sensors, actuators)
   * software architecture set-up (components, priorities, links)
   * calibration (ACC / MAG, magnetic north, neutral attitude)
   * geometry and controller configurations (pid gains)
   * user settings (x vs + mode, manual vs autonomous mode, remote control set-up)


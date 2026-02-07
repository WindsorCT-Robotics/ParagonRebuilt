# RobotContainer

The `RobotContainer` class will handle instantiation of types, creating compound commands and/or triggers, binding buttons to triggers and joysticks to commands, and binding commands to triggers.

It will also handle error handling. Library functions should bubble exceptions up and not handle them unless a library being consumed uses exceptions for process flow (which should never be done).

Default commands should also be set up in the `RobotContainer` based on operational requirements (e.g., autonomous, sim)

Robot.java should only call methods exposed by `RobotContainer`. `RobotContainer` should contain specializations for sim; if necessary, `RobotContainer` itself may be subclassed into a type that handles sim-specific behavior and hardware classes.
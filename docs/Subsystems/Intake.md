# Intake Subsystem

The Intake subsystem will have two motors. One to open the intake door and one to spin the motor 
to actually pick up Fuel.

## Dependencies

Requires two motors that can both control their duty cycle and the ability to stop and SubsystemBase.

## Public APIs

## Code Structure

### Commands

#### openDoor

`openDoor` Uses one motor to open intake door.

#### closeDoor

`closeDoor` Uses same motor as `openDoor` to close intake door.

#### moveRollers

`moveRollers` Rolls top roller in a clockwise rotation and bottom roller in a counterclockwise 
rotation.

#### reverseRollers

`reverseRollers` Rolls top roller in a counterclockwise rotation and bottom roller in a clockwise 
rotation.

#### loadFuel

`loadFuel` Uses same motor as `openDoor` and `closeDoor` to open the intake door and then uses 
second motor to spin the spindexer to load fuel balls.

#### shuttleFuel

`shuttleFuel` Uses same motor as `openDoor` and `closeDoor` to open the intake door and then uses 
second motor to spin the spindexer in reverse to shuttle out fuel balls.
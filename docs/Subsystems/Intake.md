# Intake Subsystem

The Intake subsystem will have two motors. Motor1 will open the intake door and Motor2 will spin the motor to actually pick up Fuel.

## Dependencies

Requires two motors that can both control their duty cycle.

## Public APIs

## Code Structure

### Commands

#### openDoor

`openDoor` Uses Motor1 to open intake door.

#### closeDoor

`closeDoor` Uses Motor1 to close intake door.

#### moveRollers

`moveRollers` Motor2 spins clockwise to spin top rollers counterclockwise and bottom roller 
counterclockwise.

#### reverseRollers

`reverseRollers` Motor2 spins counterclockwise to spin top rollers counterclockwise and bottom roller clockwise.

#### loadFuel

`loadFuel` Uses Motor1 to open the intake door and then uses Motor2 to spin the spindexer to load fuel balls.

#### shuttleFuel

`shuttleFuel` Uses Motor1 to open the intake door and then uses Motor2 to spin the spindexer in reverse to shuttle fuel balls.
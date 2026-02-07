# Intake Subsystem

The Intake subsystem will have two motors. DoorMotor will open the intake door and IntakeMotor will spin the motor to actually pick up Fuel.

## Dependencies

Requires two motors that can both control their duty cycle.

## Public APIs

## Code Structure

### Commands

#### openDoor

`openDoor` Uses DoorMotor to open intake door.

#### closeDoor

`closeDoor` Uses DoorMotor to close intake door.

#### moveRollers

`moveRollers` IntakeMotor spins clockwise to spin top rollers clockwise and bottom roller 
counterclockwise.

#### reverseRollers

`reverseRollers` IntakeMotor spins counterclockwise to spin top rollers counterclockwise and bottom roller clockwise.

#### loadFuel

`loadFuel` Uses DoorMotor to open the intake door and then uses IntakeMotor to spin the spindexer to load fuel.

#### shuttleFuel

`shuttleFuel` Uses DoorMotor to open the intake door and then uses IntakeMotor to spin the spindexer in reverse to shuttle fuel.
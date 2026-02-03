# Spindexer Subsystem

## Dependencies

Requires a motor that can control its duty cycle and the ability to stop.

## Public APIs

### Commands

#### moveBall

`moveBall` will turn in the correct orientation by setting the duty cycle of the motor.

`moveBall` doesn't require any parameters.

### Triggers

#### isMotorTurning

`isMotorTurning` will return a boolean that checks if the motor's angular velocity is greater than 0.
# Shooter Subsystem

The Shooter subsystem will use three motors that will spin in unison in order to launch fuel

## Dependencies
Main shooting mechanism that requires three motors that can spin by a dutyCycle. If one motor preforms the action the other motor must perform the same action. The main motor must follow the other motors in hardware.

The Kicker requires a motor that can be controlled with a dutyCycle.

## Public APIs

### Commands

#### shoot

The `Shooting` command should take in 2 different `Supplier<LinearVelocity>` for main shooting mechanism and the kicker.

The `Shooting` command will use the 2 LinearVelocities to execute a set targetSpeed to the main shooting mechanism and the kicker respectively.

##### shootWithTrajectoryPrediction

Currently unable to determine how this command will work.
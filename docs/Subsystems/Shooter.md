# Shooter Subsystem

## Dependencies
Main shooting mechanism that requires two motors that can spin by a dutyCycle. If one motor perofrma the action the other motor must perform the same action. The main motor must follow the other motor in hardware.

The Kicker requires a motor that can be controlled with a dutyCycle.

## API

### Commands

#### shoot

The `Shooting` command should take in 2 different `Supplier<LinearVelocity>` for main shooting mechanism and the kicker.

The `Shooting` command will use the 2 LinearVelocities to execute a set targetSpeed to the main shooting mechanism and the kicker respectively.

##### shootWithTrajectoryPrediction

Currently unable to determine how this command will work.
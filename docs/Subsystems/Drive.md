# Drive Subsystem

The Drive subsystem controls the swerve drive mechanism on the robot to drive it in the desired direction and angle at the desired speed.

## Dependencies

Requires the auto-generated swervedrive code.

## Code Structure

Nothing currently unique to the code structure.

## Public API

### Commands

#### Drive

##### move

The `move` command will take in a supplier of `LinearVelocity` x and y, supplier of `AngularVelocity`, and a `RelativeReference`.

Depending on the `RelativeReference` will decide if `move` will return a run of `RobotCentric` or `FieldCentric` swerve request.

##### moveWithLockedAngle

The `moveWithLockedAngle` command will take in a supplier of `LinearVelocity` x and y and a supplier `Angle` for the desired angle to lock on to.

`moveWithLockedAngle` will execute a `SwerveRequest` of `FieldCentricFacingAngle` with proper PID constants and with `withTargetDirection` for the desired angle

##### driveTorqueBased

Reference: https://pathplanner.dev/pplib-swerve-setpoint-generator.html

The `driveTorqueBased` command will take in `ChassisSpeeds`, `RelativeReference`, suppliers of `LinearVelocity` x and y, and supplier of `AngularVelocity`.

Execution of `sequentialCommand`:

1. `move`
2.`driveTorqueBased` will generate a previoudSetpoint with `generateSetpoint` and with the generated setpoint, pass it in to the method `previousSetPoint` as `setpoint.moduleStates()`.

Then `generateSetpoint` method will take in the previous setpoint before the one generated, `ChassisSpeeds`, and the loop time of robot code through `TimedRobot.kDefaultPeriod`.

#### Pathing

##### pathToPose

The `pathToPose` command will create a path that will path from robot position to the position designated.

###### pathToHubTrajectory

The `pathToHubTrajectory` command will find the fastest position to score and then use `pathToPose` to path.
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

`move` will be able to turn and move the robot with based on a reference of either `robotCentric` or `fieldCentric`.

###### robotCentricMove

`robotCentricMove` will be able to turn and move the robot with a `robotCentric` swerve request.

###### fieldCentricMove

`fieldCentricMove` will be able to turn and move the robot with a `fieldCentric` swerve request.

##### moveWithPercentages

`moveWithPercentages` will be able to turn and move the robot with `move` with the reference of either `robotCentric` or `fieldCentric` with percentages.

##### moveWithLockedAngle

`moveWithLockedAngle` will be able to move the robot with a targeted angle that the robot will try to face.

##### angleToOutpost

`angleToOutpost` will be able to move the robot with a fixed angle based on alliance with the robot's back facing the outpost.

##### driveTorqueBased

Reference: https://pathplanner.dev/pplib-swerve-setpoint-generator.html

The `driveTorqueBased` command will take in `ChassisSpeeds`, `RelativeReference`, suppliers of `LinearVelocity` x and y, and supplier of `AngularVelocity`.

Execution of `sequentialCommand`:

1. `move`
2.`driveTorqueBased` will generate a previoudSetpoint with `generateSetpoint` and with the generated setpoint, pass it in to the method `previousSetPoint` as `setpoint.moduleStates()`.

Then `generateSetpoint` method will take in the previous setpoint before the one generated, `ChassisSpeeds`, and the loop time of robot code through `TimedRobot.kDefaultPeriod`.

#### Pathing

##### pathToPosition

`pathToPosition` will be able to move towards a position.

###### createPathToPosition

`createPathToPosition` will create a `PathPlannerPath` to pass into pathToPosition.
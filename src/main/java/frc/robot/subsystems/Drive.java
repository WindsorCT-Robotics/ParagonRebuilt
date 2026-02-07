package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.GeneratedDrive;
import frc.robot.result.Failure;
import frc.robot.result.Result;
import frc.robot.result.Success;

public class Drive extends GeneratedDrive {
    // TODO: Max velocities should be properly tested.
    private static final LinearVelocity MAX_LINEAR_VELOCITY = MetersPerSecond.of(1);
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(1);
    private static final PIDConstants DEFAULT_TARGET_DIRECTION_PID = new PIDConstants(7, 0, 0);
    private static final PIDConstants DEFAULT_TRANSLATION_PID = new PIDConstants(10);
    private static final PIDConstants DEFAULT_ROTATION_PID = new PIDConstants(7);
    private static final Angle ALLIANCE_BLUE_SIDE = Degrees.of(0.0);
    private static final Angle ALLIANCE_RED_SIDE = Degrees.of(180.0);

    private final RobotConfig robotConfiguration;
    private final SwerveRequest.ApplyRobotSpeeds pathPlannerSwerveRequest = new SwerveRequest.ApplyRobotSpeeds();

    public sealed interface CommandError permits AllianceUnknown {
    }

    public record AllianceUnknown() implements CommandError {
    }

    public Drive(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) throws IOException, ParseException {

        super(drivetrainConstants, modules);

        robotConfiguration = RobotConfig.fromGUISettings();

        AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose,
                () -> getState().Speeds,
                (speeds, feedforwards) -> robotCentricChassisSpeedsMove(speeds, feedforwards),
                new PPHolonomicDriveController(
                        DEFAULT_TRANSLATION_PID,
                        DEFAULT_ROTATION_PID),
                robotConfiguration,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    private void robotCentricChassisSpeedsMove(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        setControl(pathPlannerSwerveRequest
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
    }

    private enum RelativeReference {
        ROBOT_RELATIVE,
        FIELD_RELATIVE
    }

    private Command robotCentricMove(
            Supplier<LinearVelocity> x,
            Supplier<LinearVelocity> y,
            Supplier<AngularVelocity> rotateRate) {
        return applyRequest(() -> new RobotCentric()
                .withVelocityX(x.get())
                .withVelocityY(y.get())
                .withRotationalRate(rotateRate.get()));
    }

    private Command fieldCentricMove(
            Supplier<LinearVelocity> x,
            Supplier<LinearVelocity> y,
            Supplier<AngularVelocity> rotateRate) {
        return run(() -> setControl(
                new FieldCentric()
                        .withVelocityX(x.get())
                        .withVelocityY(y.get())
                        .withRotationalRate(rotateRate.get())));
    }

    public Command move(
            Supplier<LinearVelocity> x,
            Supplier<LinearVelocity> y,
            Supplier<AngularVelocity> rotateRate,
            RelativeReference reference) {
        switch (reference) {
            case ROBOT_RELATIVE:
                return robotCentricMove(x, y, rotateRate);
            case FIELD_RELATIVE:
                return fieldCentricMove(x, y, rotateRate);
            default:
                throw new IllegalArgumentException(
                        "Unable to determine the SwerveRequest return. Illegal RelativeReference: " + reference);
        }
    }

    private LinearVelocity percentageToLinearVelocity(LinearVelocity velocity, Supplier<Dimensionless> percent) {
        return velocity.times(percent.get());
    }

    private AngularVelocity percentToAngularVelocity(AngularVelocity velocity, Supplier<Dimensionless> percent) {
        return velocity.times(percent.get());
    }

    public Command moveWithPercentages(
            Supplier<Dimensionless> x,
            Supplier<Dimensionless> y,
            Supplier<Dimensionless> rotateRate,
            RelativeReference reference) {
        return move(
                () -> percentageToLinearVelocity(MAX_LINEAR_VELOCITY, x),
                () -> percentageToLinearVelocity(MAX_LINEAR_VELOCITY, y),
                () -> percentToAngularVelocity(MAX_ANGULAR_VELOCITY, rotateRate),
                reference);
    }

    /**
     * Moves with the ability to control rotation with with a target angle.
     * 
     * @param x
     * @param y
     * @param targetAngle
     */
    private void moveWithLockedAngle(
            LinearVelocity x,
            LinearVelocity y,
            Angle targetAngle) {
        setControl(
                new FieldCentricFacingAngle()
                        .withVelocityX(x)
                        .withVelocityY(y)
                        .withHeadingPID(DEFAULT_TARGET_DIRECTION_PID.kP, DEFAULT_TARGET_DIRECTION_PID.kI,
                                DEFAULT_TARGET_DIRECTION_PID.kD)
                        .withTargetDirection(new Rotation2d(targetAngle.in(Degrees))));
    }

    public Result<Command, CommandError> angleToOutpost(
            Supplier<LinearVelocity> x,
            Supplier<LinearVelocity> y) {

        if (DriverStation.getAlliance().isEmpty()) {
            return new Failure<>(new AllianceUnknown());
        }

        return new Success<>(
                run(() -> {
                    Alliance alliance = DriverStation.getAlliance().orElseThrow();
                    Angle targetAngle;

                    if (alliance.equals(Alliance.Blue)) {
                        targetAngle = ALLIANCE_BLUE_SIDE;
                    } else {
                        targetAngle = ALLIANCE_RED_SIDE;
                    }

                    moveWithLockedAngle(x.get(), y.get(), targetAngle);
                }));
    }

    private PathPlannerPath createPathToPosition(
            Pose3d endPosition3d,
            PathConstraints constraints,
            IdealStartingState idealStartingState,
            GoalEndState goalEndState,
            Distance robotNextControlDistance,
            Distance endAnchorPreviousControlDistance) {
        SwerveDriveState robotState = getState();
        ChassisSpeeds fieldRelativeChassisSpeeds = robotState.Speeds;
        Angle directionOfVelocity = Radians.of(
                Math.atan2(fieldRelativeChassisSpeeds.vyMetersPerSecond, fieldRelativeChassisSpeeds.vxMetersPerSecond));

        Translation2d robotAnchor = new Translation2d(robotState.Pose.getMeasureX(), robotState.Pose.getMeasureY());
        Translation2d robotNextControl = new Translation2d(robotNextControlDistance.in(Meters),
                new Rotation2d(directionOfVelocity));
        Waypoint robotPositionWaypoint = new Waypoint(new Translation2d(), robotAnchor, robotNextControl);

        Pose2d endPosition2d = endPosition3d.toPose2d();
        Translation2d endAnchor = new Translation2d(endPosition2d.getMeasureX(), endPosition2d.getMeasureY());
        Translation2d endPreviousControl = new Translation2d(endAnchorPreviousControlDistance.in(Meters),
                goalEndState.rotation());
        Waypoint endPositionWaypoint = new Waypoint(endPreviousControl, endAnchor, new Translation2d());

        List<Waypoint> waypoints = List.of(robotPositionWaypoint, endPositionWaypoint);
        return new PathPlannerPath(waypoints, constraints, idealStartingState, goalEndState);
    }

    public Command pathToPosition(
            Pose3d endPosition3d,
            PathConstraints constraints,
            IdealStartingState idealStartingState,
            GoalEndState goalEndState,
            Distance robotNextControlDistance,
            Distance endAnchorPreviousControlDistance) {
        return AutoBuilder.followPath(
                createPathToPosition(
                        endPosition3d,
                        constraints,
                        idealStartingState,
                        goalEndState,
                        robotNextControlDistance,
                        endAnchorPreviousControlDistance));
    }
}
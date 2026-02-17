package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;
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
import frc.robot.generated.TunerConstants;

public class Drive extends GeneratedDrive {
    // TODO: Max velocities should be properly tested.
    private static final LinearVelocity MAX_LINEAR_VELOCITY = TunerConstants.kSpeedAt12Volts;
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.75);
    private static final PIDConstants DEFAULT_TARGET_DIRECTION_PID = new PIDConstants(7, 0, 0);
    private static final PIDConstants DEFAULT_TRANSLATION_PID = new PIDConstants(10);
    private static final PIDConstants DEFAULT_ROTATION_PID = new PIDConstants(7);
    private static final Angle ALLIANCE_BLUE_SIDE = Degrees.of(0.0);
    private static final Angle ALLIANCE_RED_SIDE = Degrees.of(180.0);

    private final RobotConfig robotConfiguration;
    private final SwerveRequest.ApplyRobotSpeeds pathPlannerSwerveRequest = new SwerveRequest.ApplyRobotSpeeds();

    public Drive(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) throws IOException, ParseException {

        super(drivetrainConstants, modules);

        // TODO: Properly set GUI Settings in PathPlanner
        robotConfiguration = RobotConfig.fromGUISettings();

        AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose,
                () -> getState().Speeds,
                this::robotCentricChassisSpeedsMove,
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

    public enum RelativeReference {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    private SwerveRequest robotCentricSwerveRequest(
            LinearVelocity x,
            LinearVelocity y,
            AngularVelocity rotateRate) {
        return new RobotCentric()
                .withVelocityX(x)
                .withVelocityY(y)
                .withRotationalRate(rotateRate);
    }

    private SwerveRequest fieldCentricSwerveRequest(
            LinearVelocity x,
            LinearVelocity y,
            AngularVelocity rotateRate) {
        return new FieldCentric()
                .withVelocityX(x)
                .withVelocityY(y)
                .withRotationalRate(rotateRate);
    }

    public Command move(
            Supplier<LinearVelocity> x,
            Supplier<LinearVelocity> y,
            Supplier<AngularVelocity> rotateRate,
            Supplier<RelativeReference> reference) {
        return run(() -> {
            switch (reference.get()) {
                case ROBOT_CENTRIC:
                    robotCentricSwerveRequest(x.get(), y.get(), rotateRate.get());
                    break;
                case FIELD_CENTRIC:
                    fieldCentricSwerveRequest(x.get(), y.get(), rotateRate.get());
                    break;
                default:
                    throw new IllegalArgumentException(
                            "Unable to determine the SwerveRequest return. Illegal RelativeReference: "
                                    + reference.get().toString());
            }
        });
    }

    private LinearVelocity percentageToLinearVelocity(LinearVelocity velocity, Supplier<Dimensionless> percent) {
        return velocity.times(percent.get());
    }

    private AngularVelocity percentageToAngularVelocity(AngularVelocity velocity, Supplier<Dimensionless> percent) {
        return velocity.times(percent.get());
    }

    public Command moveWithPercentages(
            Supplier<Dimensionless> x,
            Supplier<Dimensionless> y,
            Supplier<Dimensionless> rotateRate,
            Supplier<RelativeReference> reference) {
        return move(
                () -> percentageToLinearVelocity(MAX_LINEAR_VELOCITY, x),
                () -> percentageToLinearVelocity(MAX_LINEAR_VELOCITY, y),
                () -> percentageToAngularVelocity(MAX_ANGULAR_VELOCITY, rotateRate),
                reference);
    }

    /**
     * Moves with the ability to control rotation by a target angle WITH ONLY FIELD
     * CENTRIC.
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

    public Command angleToOutpost(
            Supplier<Dimensionless> x,
            Supplier<Dimensionless> y) {
        return run(() -> {
            Optional<Alliance> maybeAlliance = DriverStation.getAlliance();

            maybeAlliance.ifPresent(alliance -> {
                Angle targetAngle;

                if (maybeAlliance.get().equals(Alliance.Blue)) {
                    targetAngle = ALLIANCE_BLUE_SIDE;
                } else {
                    targetAngle = ALLIANCE_RED_SIDE;
                }

                moveWithLockedAngle(
                        percentageToLinearVelocity(MAX_LINEAR_VELOCITY, x),
                        percentageToLinearVelocity(MAX_LINEAR_VELOCITY, y),
                        targetAngle);
            });
        }).unless(DriverStation.getAlliance()::isEmpty);
    }

    private PathPlannerPath createPathToPosition(
            Pose3d endPosition3d,
            PathConstraints constraints,
            GoalEndState goalEndState,
            Distance robotNextControlDistance,
            Distance endAnchorPreviousControlDistance) {
        Pigeon2 gyro = getPigeon2();
        SwerveDriveState robotState = getState();
        ChassisSpeeds robotCentricChassisSpeeds = robotState.Speeds;
        ChassisSpeeds fieldCentricChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(robotCentricChassisSpeeds,
                gyro.getRotation2d());

        LinearVelocity robotVelocity = MetersPerSecond.of(
                Math.sqrt(
                        Math.pow(robotCentricChassisSpeeds.vxMetersPerSecond, 2)
                                + Math.pow(robotCentricChassisSpeeds.vyMetersPerSecond, 2)));
        Angle directionOfVelocity = Radians.of(
                Math.atan2(fieldCentricChassisSpeeds.vyMetersPerSecond, fieldCentricChassisSpeeds.vxMetersPerSecond));

        Translation2d robotAnchor = new Translation2d(robotState.Pose.getMeasureX(), robotState.Pose.getMeasureY());
        Translation2d robotNextControl = new Translation2d(robotNextControlDistance.in(Meters),
                new Rotation2d(directionOfVelocity));
        Waypoint robotPositionWaypoint = new Waypoint(new Translation2d(), robotAnchor, robotNextControl);

        Pose2d endPosition2d = endPosition3d.toPose2d();
        Translation2d endAnchor = new Translation2d(endPosition2d.getMeasureX(), endPosition2d.getMeasureY());
        Translation2d endPreviousControl = new Translation2d(endAnchorPreviousControlDistance.in(Meters),
                goalEndState.rotation());
        Waypoint endPositionWaypoint = new Waypoint(endPreviousControl, endAnchor, new Translation2d());

        IdealStartingState idealStartingState = new IdealStartingState(robotVelocity,
                new Rotation2d(directionOfVelocity));
        List<Waypoint> waypoints = List.of(robotPositionWaypoint, endPositionWaypoint);
        return new PathPlannerPath(waypoints, constraints, idealStartingState, goalEndState);
    }

    public Command pathToPosition(
            Pose3d endPosition3d,
            PathConstraints constraints,
            GoalEndState goalEndState,
            Distance robotNextControlDistance,
            Distance endAnchorPreviousControlDistance) {
        return AutoBuilder.followPath(
                createPathToPosition(
                        endPosition3d,
                        constraints,
                        goalEndState,
                        robotNextControlDistance,
                        endAnchorPreviousControlDistance));
    }
}
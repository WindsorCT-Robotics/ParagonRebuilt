package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.GeneratedDrive;
import frc.robot.generated.LimelightHelpers;
import frc.robot.generated.RectanglePoseArea;
import frc.robot.generated.TunerConstants;

public class Drive extends GeneratedDrive implements Sendable {
        // TODO: Max velocities should be properly tested.
        private static final LinearVelocity MAX_LINEAR_VELOCITY = TunerConstants.kSpeedAt12Volts;
        private static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.75);
        private static final PIDConstants DEFAULT_TARGET_DIRECTION_PID = new PIDConstants(0.5, 0, 0.001);
        private PIDConstants targetPID = new PIDConstants(0.3, 0.0, 0.0);
        private static final PIDConstants DEFAULT_TRANSLATION_PID = new PIDConstants(10);
        private static final PIDConstants DEFAULT_ROTATION_PID = new PIDConstants(7);
        private static final Angle ALLIANCE_BLUE_SIDE = Degrees.of(0.0);
        private static final Angle ALLIANCE_RED_SIDE = Degrees.of(180.0);
        private static final Angle SHOOTER_OFFSET = Degrees.of(90);
        private final String limelightName;
        private final RectanglePoseArea field;

        private final RobotConfig robotConfiguration;
        private final SwerveRequest.ApplyRobotSpeeds pathPlannerSwerveRequest = new SwerveRequest.ApplyRobotSpeeds();

        public Drive(
                        String name,
                        String limelightName,
                        SwerveDrivetrainConstants drivetrainConstants,
                        SwerveModuleConstants<?, ?, ?>... modules) throws IOException, ParseException {
                super(drivetrainConstants, modules);
                SendableRegistry.addLW(this, "Subsystems/" + name, "Subsystems/" + name);
                CommandScheduler.getInstance().registerSubsystem(this);

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
                this.limelightName = limelightName;
                AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
                this.field = new RectanglePoseArea(Translation2d.kZero,
                                new Translation2d(layout.getFieldWidth(), layout.getFieldLength()));
                setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));

                LimelightHelpers.setCameraPose_RobotSpace(
                                limelightName,
                                Meters.of(0.1778).in(Meters),
                                Meters.of(0.0635).in(Meters),
                                Meters.of(0.5588).in(Meters),
                                Degrees.zero().in(Degrees),
                                Degrees.of(30).in(Degrees),
                                Degrees.of(-90).in(Degrees));
                LimelightHelpers.SetRobotOrientation(limelightName,
                                getPigeon2().getYaw().getValue().in(Degrees),
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0);

                initSmartDashboard();
        }

        private Angle getAngle() {
                return getPigeon2().getYaw().getValue();
        }

        @Override
        public void periodic() {
                super.periodic();
                updateLimelightOrientationToRobot();
                addVisionMeasurements();
        }

        private void initSmartDashboard() {
                SmartDashboard.putData(getName(), this);
                SmartDashboard.putData(getName() + "/" + getPigeon2().getClass().getSimpleName(),
                                getPigeon2());
                // https://frc-elastic.gitbook.io/docs/additional-features-and-references/custom-widget-examples#swervedrive
                // TODO: See if these values are correct.
                SmartDashboard.putData(getName() + "/SwerveDrive", new Sendable() {
                        @Override
                        public void initSendable(SendableBuilder builder) {
                                builder.setSmartDashboardType("SwerveDrive");

                                builder.addDoubleProperty("Front Left Angle",
                                                () -> getState().ModulePositions[0].angle.getRadians(),
                                                null);
                                builder.addDoubleProperty("Front Left Velocity",
                                                () -> getState().ModuleTargets[0].speedMetersPerSecond,
                                                null);

                                builder.addDoubleProperty("Front Right Angle",
                                                () -> getState().ModulePositions[1].angle.getRadians(),
                                                null);
                                builder.addDoubleProperty("Front Right Velocity",
                                                () -> getState().ModuleTargets[1].speedMetersPerSecond, null);

                                builder.addDoubleProperty("Back Left Angle",
                                                () -> getState().ModulePositions[2].angle.getRadians(),
                                                null);
                                builder.addDoubleProperty("Back Left Velocity",
                                                () -> getState().ModuleTargets[2].speedMetersPerSecond,
                                                null);

                                builder.addDoubleProperty("Back Right Angle",
                                                () -> getState().ModulePositions[3].angle.getRadians(),
                                                null);
                                builder.addDoubleProperty("Back Right Velocity",
                                                () -> getState().ModuleTargets[3].speedMetersPerSecond,
                                                null);
                        }
                });

        }

        private void setP(double p) {
                targetPID = new PIDConstants(p, targetPID.kI, targetPID.kD);
        }

        private double getP() {
                return targetPID.kP;
        }

        private void setI(double i) {
                targetPID = new PIDConstants(targetPID.kI, i, targetPID.kD);
        }

        private double getI() {
                return targetPID.kI;
        }

        private void setD(double d) {
                targetPID = new PIDConstants(targetPID.kP, targetPID.kI, d);
        }

        private double getD() {
                return targetPID.kD;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Subsystem");

                builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
                builder.addStringProperty(
                                ".default",
                                () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
                                null);
                builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
                builder.addStringProperty(
                                ".command",
                                () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
                                null);

                builder.addDoubleProperty("Target P", this::getP, this::setP);
                builder.addDoubleProperty("Target I", this::getI, this::setI);
                builder.addDoubleProperty("Target D", this::getD, this::setD);
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

        // TODO: Forward is intake side.
        private SwerveRequest robotCentricSwerveRequest(
                        LinearVelocity x,
                        LinearVelocity y,
                        AngularVelocity rotateRate) {
                return new RobotCentric()
                                .withVelocityX(y)
                                .withVelocityY(x)
                                .withRotationalRate(rotateRate)
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        }

        private SwerveRequest fieldCentricSwerveRequest(
                        LinearVelocity x,
                        LinearVelocity y,
                        AngularVelocity rotateRate) {
                return new FieldCentric()
                                .withVelocityX(y)
                                .withVelocityY(x)
                                .withRotationalRate(rotateRate)
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        }

        public Command move(
                        Supplier<LinearVelocity> x,
                        Supplier<LinearVelocity> y,
                        Supplier<AngularVelocity> rotateRate,
                        Supplier<RelativeReference> reference) {
                return run(() -> {
                        switch (reference.get()) {
                                case ROBOT_CENTRIC:
                                        setControl(robotCentricSwerveRequest(x.get().unaryMinus(), y.get().unaryMinus(),
                                                        rotateRate.get()));
                                        break;
                                case FIELD_CENTRIC:
                                        setControl(fieldCentricSwerveRequest(x.get(), y.get(), rotateRate.get()));
                                        break;
                                default:
                                        throw new IllegalArgumentException(
                                                        "Unable to determine the SwerveRequest return. Illegal RelativeReference: "
                                                                        + reference.get().toString());
                        }
                }).withName("Subsystems/" + getName() + "/moveBayDoorTo");
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
                                reference).withName("Subsystems/" + getName() + "/moveWithPercentages");
        }

        /**
         * Moves with the ability to control rotation by a target angle WITH ONLY FIELD
         * CENTRIC.
         */
        private void moveWithLockedAngle(
                        LinearVelocity x,
                        LinearVelocity y,
                        Angle targetAngle,
                        Angle threshold) {
                Angle robotHeading = Radians.of(MathUtil.angleModulus(getAngle().in(Radians) + Math.PI));

                SmartDashboard.putNumber("Robot Heading Wrapped", robotHeading.in(Degrees));
                SmartDashboard.putBoolean("Should Correct Robot Heading", !robotHeading.isNear(targetAngle, threshold));

                if (!robotHeading.isNear(targetAngle, threshold)) {
                        setControl(
                                        new FieldCentricFacingAngle()
                                                        .withVelocityX(y)
                                                        .withVelocityY(x)
                                                        .withHeadingPID(targetPID.kP,
                                                                        targetPID.kI,
                                                                        targetPID.kD)
                                                        .withDriveRequestType(DriveRequestType.Velocity)
                                                        .withTargetDirection(new Rotation2d(targetAngle.in(Radians))));
                } else {
                        setControl(fieldCentricSwerveRequest(x, y, RPM.zero()));
                }

                SmartDashboard.putNumber("Target Angle", targetAngle.in(Degrees));
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
                                                targetAngle,
                                                Degrees.of(5));
                        });
                }).withName("Subsystems/" + getName() + "/angleToOutpost")
                                .unless(DriverStation.getAlliance()::isEmpty);
        }

        private Pose2d getHubPosition(Alliance alliance) {
                AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
                Pose3d blueHubYCenter = layout.getTagPose(26).get();
                Pose3d blueHubXCenter = layout.getTagPose(21).get();
                Pose3d redHubYCenter = layout.getTagPose(10).get();
                Pose3d redHubXCenter = layout.getTagPose(5).get();

                Distance xHub;
                Distance yHub;
                if (alliance.equals(Alliance.Blue)) {
                        xHub = blueHubXCenter.getMeasureX();
                        yHub = blueHubYCenter.getMeasureY();
                } else {
                        xHub = redHubXCenter.getMeasureX();
                        yHub = redHubYCenter.getMeasureY();
                }

                return new Pose2d(xHub, yHub, new Rotation2d());
        }

        public Command angleToHub(
                        Supplier<Dimensionless> x,
                        Supplier<Dimensionless> y) {
                return run(() -> {
                        Optional<Alliance> maybeAlliance = DriverStation.getAlliance();

                        maybeAlliance.ifPresent(alliance -> {
                                Pose2d robotPosition = getState().Pose;
                                Pose2d hubPosition = getHubPosition(alliance);
                                Distance xDifference = robotPosition.getMeasureX().minus(hubPosition.getMeasureX());
                                Distance yDifference = robotPosition.getMeasureY().minus(hubPosition.getMeasureY());

                                Angle targetAngle = Radians
                                                .of(Math.atan2(yDifference.in(Meters), xDifference.in(Meters)))
                                                .plus(SHOOTER_OFFSET);

                                Angle wrapTargetAngle = Radians.of(MathUtil.angleModulus(targetAngle.in(Radians)));

                                moveWithLockedAngle(
                                                percentageToLinearVelocity(MAX_LINEAR_VELOCITY, x),
                                                percentageToLinearVelocity(MAX_LINEAR_VELOCITY, y),
                                                wrapTargetAngle,
                                                Degrees.of(2));
                        });
                });
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
                ChassisSpeeds fieldCentricChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                robotCentricChassisSpeeds,
                                gyro.getRotation2d());

                LinearVelocity robotVelocity = MetersPerSecond.of(
                                Math.sqrt(
                                                Math.pow(robotCentricChassisSpeeds.vxMetersPerSecond, 2)
                                                                + Math.pow(robotCentricChassisSpeeds.vyMetersPerSecond,
                                                                                2)));
                Angle directionOfVelocity = Radians.of(
                                Math.atan2(fieldCentricChassisSpeeds.vyMetersPerSecond,
                                                fieldCentricChassisSpeeds.vxMetersPerSecond));

                Translation2d robotAnchor = new Translation2d(robotState.Pose.getMeasureX(),
                                robotState.Pose.getMeasureY());
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
                                                endAnchorPreviousControlDistance))
                                .withName("Subsystems/" + getName() + "/pathToPosition");
        }

        public Command resetGyro() {
                return Commands.runOnce(() -> getPigeon2().setYaw(Degrees.of(0.0)))
                                .withName("Subsystems/" + getName() + "/resetGyro");
        }

        private LimelightHelpers.PoseEstimate getPositionEstimate() {
                return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        }

        private void addVisionMeasurements() {
                double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getTranslation()
                                .getDistance(new Translation3d()); // Calculates how far away the april tag is
                double confidence = (targetDistance - 1) / 6;
                LimelightHelpers.PoseEstimate positionEstimate = getPositionEstimate();
                Pose2d position = positionEstimate.pose;
                SmartDashboard.putNumber("Vision Position X", position.getX());
                SmartDashboard.putNumber("Vision Position Y", position.getY());
                if (getPositionEstimate().tagCount <= 0)
                        return;
                if (!field.isPoseWithinArea(positionEstimate.pose))
                        return;
                addVisionMeasurement(positionEstimate.pose, positionEstimate.timestampSeconds,
                                VecBuilder.fill(confidence, confidence, 0.1));
        }

        private void updateLimelightOrientationToRobot() {
                LimelightHelpers.SetRobotOrientation(limelightName, getAngle().in(Degrees), 0.0, 0.0, 0.0, 0.0, 0.0);
        }
}
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.GeneratedDrive;
import frc.robot.generated.LimelightHelpers;
import frc.robot.generated.RectanglePoseArea;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.LimelightHelpers.PoseEstimate;

public class Drive extends GeneratedDrive implements Sendable {
        // region
        private static final LinearVelocity MAX_LINEAR_VELOCITY = TunerConstants.kSpeedAt12Volts;
        private static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.75);
        private static final PIDConstants FACING_ANGLE_PID = new PIDConstants(10, 0, 0.5);
        private static final PIDConstants DEFAULT_TRANSLATION_PID = new PIDConstants(1);
        private static final PIDConstants DEFAULT_ROTATION_PID = new PIDConstants(2);
        private static final Distance LAUNCHER_DISTANCE_FROM_ROBOT = Inches.of(11.3);
        private static final Distance LAUNCHER_TANGENT_OFFSET = LAUNCHER_DISTANCE_FROM_ROBOT
                        .times(Math.cos(Degrees.of(45).in(Radians)));
        private static final Angle LAUNCHER_ANGLE_OFFSET = Degrees.of(90);

        private static final AprilTagFieldLayout layout = AprilTagFieldLayout
                        .loadField(AprilTagFields.k2026RebuiltAndymark);
        private static final Distance HALF_FIELD_Y = Meters.of(layout.getFieldWidth()).div(2);
        private static final Distance UPPER_NET_Y = Inches.of(188.045);
        private static final Distance BOTTOM_NET_Y = Inches.of(129.635);

        private final Distance blueHubBackSideX = Inches.of(216.13);
        private final Distance redHubBackSideX = Inches.of(435.09);
        private final Pose3d blueHubYCenter = layout.getTagPose(26).get();
        private final Pose3d blueHubXCenter = layout.getTagPose(21).get();
        private final Pose3d redHubYCenter = layout.getTagPose(10).get();
        private final Pose3d redHubXCenter = layout.getTagPose(5).get();
        private final Translation2d blueHub = new Translation2d(blueHubXCenter.getMeasureX(),
                        blueHubYCenter.getMeasureY());
        private final Translation2d redHub = new Translation2d(redHubXCenter.getMeasureX(),
                        redHubYCenter.getMeasureY());

        private final Pose3d redOffsetOutpost = layout.getTagPose(14).get();
        private final Pose3d blueOffsetOutpost = layout.getTagPose(30).get();
        private final Translation2d blueSnowblowLeft = new Translation2d(blueOffsetOutpost.getMeasureX(),
                        redOffsetOutpost.getMeasureY());
        private final Translation2d blueSnowblowRight = blueOffsetOutpost.toPose2d().getTranslation();
        private final Translation2d redSnowblowLeft = redOffsetOutpost.toPose2d().getTranslation();
        private final Translation2d redSnowblowRight = new Translation2d(redOffsetOutpost.getMeasureX(),
                        blueOffsetOutpost.getMeasureY());

        private static final NetworkTableInstance NT_INSTANCE = NetworkTableInstance.getDefault();

        private final NetworkTable driveTable = NT_INSTANCE.getTable("SmartDashboard/Subsystems/Drive");
        private final StructPublisher<Pose2d> robotPosition = driveTable
                        .getStructTopic("Robot Position 2D", Pose2d.struct).publish();
        private final StructPublisher<Pose2d> invalidVisionPosition = driveTable
                        .getStructTopic("Invalid Vision Position 2D", Pose2d.struct).publish();
        private final StructPublisher<Pose2d> validvisionPosition = driveTable
                        .getStructTopic("Valid Vision Position 2D", Pose2d.struct).publish();
        private final StructArrayPublisher<SwerveModuleState> currentModulesStates = driveTable
                        .getStructArrayTopic("Current Modules States", SwerveModuleState.struct).publish();
        private final StructArrayPublisher<SwerveModuleState> targetModuleStates = driveTable
                        .getStructArrayTopic("Target Modules States", SwerveModuleState.struct).publish();

        private final String limelightName;
        private final RectanglePoseArea field;
        private final RobotConfig robotConfiguration;
        private final SwerveRequest.ApplyRobotSpeeds pathPlannerSwerveRequest = new SwerveRequest.ApplyRobotSpeeds();
        private final FieldCentric fieldCentricSwerveRequest = new FieldCentric();
        private final RobotCentric robotCentricSwerveRequest = new RobotCentric();
        private final FieldCentricFacingAngle fieldCentricFacingAngleSwerveRequest = new FieldCentricFacingAngle();

        private final Supplier<PoseEstimate> poseEstimate;
        public final Trigger onAllianceSide;
        public final Trigger launcherAlignedToHub;
        public final Trigger launcherAlignedToSnowblow;
        public final Trigger launcherObstructedByHub;
        public final Trigger isVisionEstimateInField;
        public final Trigger isVisionEstimateHasTags;
        public final Trigger isVisionMeasurementValid;
        // endregion

        public Drive(
                        String name,
                        String limelightName,
                        SwerveDrivetrainConstants drivetrainConstants,
                        SwerveModuleConstants<?, ?, ?>... modules) throws IOException, ParseException {
                super(drivetrainConstants, modules);
                SendableRegistry.addLW(this, "Subsystems/" + name, "Subsystems/" + name);
                CommandScheduler.getInstance().registerSubsystem(this);

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
                                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                                this);

                this.limelightName = limelightName;
                AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

                this.field = new RectanglePoseArea(
                                Translation2d.kZero,
                                new Translation2d(layout.getFieldLength(), layout.getFieldWidth()));

                setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                LimelightHelpers.setCameraPose_RobotSpace(
                                limelightName,
                                Inches.of(-8).in(Meters),
                                Inches.of(-12).in(Meters),
                                Inches.of(20.5).in(Meters),
                                Degrees.zero().in(Degrees),
                                Degrees.of(25).in(Degrees),
                                Degrees.of(90).in(Degrees));
                LimelightHelpers.SetRobotOrientation(limelightName,
                                getPigeon2().getYaw().getValue().in(Degrees),
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0);

                poseEstimate = () -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

                isVisionEstimateInField = new Trigger(() -> field.isPoseWithinArea(poseEstimate.get().pose));
                isVisionEstimateHasTags = new Trigger(() -> poseEstimate.get().tagCount > 0);
                isVisionMeasurementValid = isVisionEstimateHasTags.and(isVisionEstimateInField);

                onAllianceSide = new Trigger(() -> {
                        Optional<Alliance> alliance = DriverStation.getAlliance();
                        Pose2d robotPosition = getState().Pose;

                        if (alliance.isEmpty())
                                return false;

                        Distance robotXMeasure = robotPosition.getMeasureX();
                        Distance hubXMeasure = getHubTarget(alliance.get()).getMeasureX();
                        if (alliance.get() == Alliance.Blue)
                                return robotXMeasure.lte(hubXMeasure);

                        if (alliance.get() == Alliance.Red)
                                return robotXMeasure.gte(hubXMeasure);

                        return false;
                });

                launcherAlignedToHub = new Trigger(() -> {
                        Optional<Rotation2d> targetAngle = getLaunchAngleToHub();
                        if (targetAngle.isEmpty()) {
                                return false;
                        }

                        return isNearTargetAngle(targetAngle.get());
                });

                launcherAlignedToSnowblow = new Trigger(() -> {
                        Optional<Rotation2d> targetAngle = getLaunchAngleToSnowblow();
                        if (targetAngle.isEmpty()) {
                                return false;
                        }

                        return isNearTargetAngle(targetAngle.get());
                });

                // TODO: Should determine if launcher is intersecting the hub.
                launcherObstructedByHub = new Trigger(this::isLauncherObstructedByHub);

                fieldCentricFacingAngleSwerveRequest.HeadingController.setTolerance(Degrees.of(5).in(Radians));
                fieldCentricFacingAngleSwerveRequest.HeadingController.setPID(
                                FACING_ANGLE_PID.kP,
                                FACING_ANGLE_PID.kI,
                                FACING_ANGLE_PID.kD);
                fieldCentricFacingAngleSwerveRequest.withDriveRequestType(DriveRequestType.Velocity);

                resetGyro();
                initSmartDashboard();
        }

        public enum RelativeReference {
                ROBOT_CENTRIC,
                FIELD_CENTRIC
        }

        @Override
        public void periodic() {
                super.periodic();
                updateLimelightOrientationToRobot();
                addVisionMeasurements();

                SwerveDriveState robotState = getState();
                robotPosition.set(robotState.Pose);
                currentModulesStates.set(robotState.ModuleStates);
                targetModuleStates.set(robotState.ModuleTargets);
        }

        // region Senables
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
                builder.addBooleanProperty("Launcher Aligned To Hub", launcherAlignedToHub, null);
                builder.addBooleanProperty("Launcher Aligned To Snowblow", launcherAlignedToSnowblow, null);
                builder.addBooleanProperty("Launcher Obstructed By Hub", launcherObstructedByHub, null);
                builder.addBooleanProperty("On Alliance Side", launcherAlignedToHub, null);
                builder.addBooleanProperty("Vison Estimate in Field", isVisionEstimateInField, null);
                builder.addBooleanProperty("Vision Tags Found", isVisionEstimateHasTags, null);
                builder.addDoubleProperty("Distance To Hub (Meters)",
                                () -> getDistanceToHub().orElse(Meters.zero()).in(Meters), null);
        }

        private void initSmartDashboard() {
                SmartDashboard.putData("Subsystems/" + getName() + "/" + getPigeon2().getClass().getSimpleName(),
                                getPigeon2());
                // https://frc-elastic.gitbook.io/docs/additional-features-and-references/custom-widget-examples#swervedrive
                SmartDashboard.putData("Subsystems/" + getName() + "/SwerveDrive", new Sendable() {
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
        // endregion

        // region Getters
        private Angle getAngle() {
                Angle rawGyroAngle = getPigeon2().getYaw().getValue();
                return Radians.of(MathUtil.angleModulus(rawGyroAngle.in(Radians)));
        }

        public SwerveModuleState[] getSwerveModuleStates() {
                return getState().ModuleStates;
        }

        public SwerveModuleState[] getSwerveModuleTargetStates() {
                return getState().ModuleTargets;
        }
        // endregion

        // region General Moving
        private void robotCentricChassisSpeedsMove(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
                setControl(pathPlannerSwerveRequest
                                .withSpeeds(speeds)
                                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
        }

        private SwerveRequest robotCentricSwerveRequest(
                        LinearVelocity x,
                        LinearVelocity y,
                        AngularVelocity rotateRate) {
                return robotCentricSwerveRequest
                                .withVelocityX(y)
                                .withVelocityY(x)
                                .withRotationalRate(rotateRate);
        }

        private SwerveRequest fieldCentricSwerveRequest(
                        LinearVelocity x,
                        LinearVelocity y,
                        AngularVelocity rotateRate) {
                return fieldCentricSwerveRequest
                                .withVelocityX(y)
                                .withVelocityY(x)
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
                                        setControl(robotCentricSwerveRequest(
                                                        x.get(),
                                                        y.get(),
                                                        rotateRate.get()));
                                        break;
                                case FIELD_CENTRIC:
                                        setControl(fieldCentricSwerveRequest(
                                                        x.get(),
                                                        y.get(),
                                                        rotateRate.get()));
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

        // endregion

        // region Angle Targeting
        public Command angleToHub(
                        Supplier<Dimensionless> x,
                        Supplier<Dimensionless> y) {
                return runEnd(() -> {
                        Optional<Rotation2d> targetAngle = getLaunchAngleToHub();
                        if (targetAngle.isEmpty()) {
                                moveWithPercentages(
                                                x,
                                                y,
                                                () -> Percent.of(0),
                                                () -> RelativeReference.FIELD_CENTRIC);
                                return;
                        }
                        moveWithLockedAngle(
                                        percentageToLinearVelocity(MAX_LINEAR_VELOCITY, x),
                                        percentageToLinearVelocity(MAX_LINEAR_VELOCITY, y),
                                        targetAngle.get().getMeasure());
                }, () -> fieldCentricSwerveRequest.withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }

        public Command angleToSnowblow(
                        Supplier<Dimensionless> x,
                        Supplier<Dimensionless> y) {
                return runEnd(() -> {
                        Optional<Rotation2d> targetAngle = getLaunchAngleToSnowblow();
                        if (targetAngle.isEmpty()) {
                                moveWithPercentages(
                                                x,
                                                y,
                                                () -> Percent.of(0),
                                                () -> RelativeReference.FIELD_CENTRIC);
                                return;
                        }
                        moveWithLockedAngle(
                                        percentageToLinearVelocity(MAX_LINEAR_VELOCITY, x),
                                        percentageToLinearVelocity(MAX_LINEAR_VELOCITY, y),
                                        targetAngle.get().getMeasure());
                }, () -> fieldCentricSwerveRequest.withDriveRequestType(DriveRequestType.OpenLoopVoltage));
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
                                fieldCentricFacingAngleSwerveRequest
                                                .withVelocityX(y)
                                                .withVelocityY(x)
                                                .withTargetDirection(
                                                                new Rotation2d(targetAngle.in(Radians))));

                SmartDashboard.putNumber("Target Angle", targetAngle.in(Degrees));
        }

        private Translation2d getHubTarget(Alliance alliance) {
                if (alliance.equals(Alliance.Blue)) {
                        return blueHub;
                } else {
                        return redHub;
                }
        }

        private Translation2d getSnowblowTarget(Alliance alliance) {
                Distance robotPositionY = getState().Pose.getMeasureY();

                if (alliance == Alliance.Blue) {
                        if (robotPositionY.gt(HALF_FIELD_Y)) {
                                return blueSnowblowRight;
                        } else {
                                return blueSnowblowLeft;
                        }
                } else {
                        if (robotPositionY.gt(HALF_FIELD_Y)) {
                                return redSnowblowLeft;
                        } else {
                                return redSnowblowRight;
                        }
                }
        }

        private Optional<Rotation2d> getLaunchAngleToSnowblow() {
                Optional<Alliance> alliance = DriverStation.getAlliance();

                if (alliance.isEmpty()) {
                        return Optional.empty();
                }

                return Optional.of(getLaunchAngleToPosition(getSnowblowTarget(alliance.get()), alliance.get()));
        }

        private Optional<Rotation2d> getLaunchAngleToHub() {
                Optional<Alliance> alliance = DriverStation.getAlliance();

                if (alliance.isEmpty()) {
                        return Optional.empty();
                }

                return Optional.of(getLaunchAngleToPosition(getHubTarget(alliance.get()), alliance.get()));
        }

        private Rotation2d getLaunchAngleToPosition(Translation2d target, Alliance alliance) {
                Translation2d robotPosition = getState().Pose.getTranslation();

                Angle launcherOffset = Radians.of(Math.asin(
                                LAUNCHER_TANGENT_OFFSET.div(Meters.of(robotPosition.getDistance(target)))
                                                .in(Value)))
                                .minus(LAUNCHER_ANGLE_OFFSET);

                Translation2d targetTranslation = robotPosition.minus(target);

                Angle targetAngle = Radians
                                .of(Math.atan2(targetTranslation.getY(), targetTranslation.getX()))
                                .minus(launcherOffset);

                if (alliance == Alliance.Red) {
                        targetAngle = targetAngle.plus(Radians.of(Math.PI));
                }

                return new Rotation2d(targetAngle);
        }

        private boolean isNearTargetAngle(Rotation2d target) {
                Optional<Alliance> alliance = DriverStation.getAlliance();

                if (alliance.isEmpty()) {
                        return false;
                }

                Angle currentLaunch = getAngle();

                if (alliance.get() == Alliance.Red) {
                        target = target.plus(new Rotation2d(Radians.of(Math.PI)));
                }

                return currentLaunch.isNear(target.getMeasure(), Degrees.of(5));
        }

        private boolean isLauncherObstructedByHub() {
                Optional<Alliance> alliance = DriverStation.getAlliance();

                if (alliance.isEmpty()) {
                        DriverStation.reportWarning("No Alliance Found.", true);
                        return true;
                }

                Translation2d robotPosition = getState().Pose.getTranslation();
                Rotation2d robotRotation = new Rotation2d(getAngle());
                Distance netLength = UPPER_NET_Y.minus(BOTTOM_NET_Y);
                Distance launcherPositionX = robotPosition.getMeasureX().plus(LAUNCHER_DISTANCE_FROM_ROBOT
                                .times(Math.cos(robotRotation.getDegrees() - Degrees.of(135).in(Degrees))));
                Distance launcherPositionY = robotPosition.getMeasureY().plus(LAUNCHER_DISTANCE_FROM_ROBOT
                                .times(Math.sin(robotRotation.getDegrees() - Degrees.of(135).in(Degrees))));
                Translation2d launcherPosition = new Translation2d(launcherPositionX, launcherPositionY);
                Distance toNetX;
                Distance toNetY;

                if (alliance.get() == Alliance.Blue) {
                        // Distance to robot to net.
                        toNetX = launcherPosition.getMeasureX().minus(blueHubBackSideX);
                        // Launcher intersection then check if that intersection is with in the upper
                        // and bottom range of net.
                        toNetY = launcherPosition.getMeasureY().minus(toNetX)
                                        .times(Math.tan(robotRotation.minus(new Rotation2d(Radians.of(Math.PI)))
                                                        .getDegrees()));

                        return netLength.isNear(toNetY, netLength.div(2));
                } else {
                        // Distance to robot to net.
                        toNetX = launcherPosition.getMeasureX().minus(redHubBackSideX);
                        // Launcher intersection then check if that intersection is with in the upper
                        // and bottom range of net.
                        toNetY = launcherPosition.getMeasureY().minus(toNetX)
                                        .times(Math.tan(robotRotation.getDegrees()));

                        return netLength.isNear(toNetY, netLength.div(2));
                }
        }
        // endregion

        // region Distance Targets
        public Distance getDistanceToTarget(Translation2d target) {
                return Meters.of(getState().Pose.getTranslation().getDistance(target));
        }

        public Optional<Distance> getDistanceToHub() {
                Optional<Alliance> alliance = DriverStation.getAlliance();

                if (alliance.isEmpty()) {
                        return Optional.empty();
                }

                return Optional.of(getDistanceToTarget(getHubTarget(alliance.get())));
        }

        public Optional<Distance> getDistanceToSnowblow() {
                Optional<Alliance> alliance = DriverStation.getAlliance();

                if (alliance.isEmpty()) {
                        return Optional.empty();
                }

                return Optional.of(getDistanceToTarget(getSnowblowTarget(alliance.get())));
        }
        // endregion

        // region PathPlanner
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

        // endregion

        // region Vision
        private void addVisionMeasurements() {
                double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getTranslation()
                                .getDistance(new Translation3d());
                double confidence = (targetDistance - 1) / 6;
                LimelightHelpers.PoseEstimate positionEstimate = poseEstimate.get();

                if (!isVisionMeasurementValid.getAsBoolean()) {
                        invalidVisionPosition.set(positionEstimate.pose);
                        return;
                }

                addVisionMeasurement(
                                positionEstimate.pose,
                                positionEstimate.timestampSeconds,
                                VecBuilder.fill(confidence, confidence, 0.1));
                validvisionPosition.set(positionEstimate.pose);
        }

        private void updateLimelightOrientationToRobot() {
                LimelightHelpers.SetRobotOrientation(limelightName, getAngle().in(Degrees),
                                0.0, 0.0, 0.0, 0.0, 0.0);
        }
        // endregion

        public void resetGyro() {
                getPigeon2().setYaw(Degrees.of(0.0));
        }

        public Command resetGyroCommand() {
                return Commands.runOnce(() -> resetGyro())
                                .withName("Subsystems/" + getName() + "/resetGyro");
        }
}
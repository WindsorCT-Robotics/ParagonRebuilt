package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
        private static final LinearVelocity MAX_LINEAR_VELOCITY = TunerConstants.kSpeedAt12Volts;
        private static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.75);
        private static final PIDConstants FACING_ANGLE_PID = new PIDConstants(0.5, 0, 0.3);
        private final ProfiledPIDController alignController = new ProfiledPIDController(0.2, 0.0, 0.0, new Constraints(
                        RotationsPerSecond.of(1).in(DegreesPerSecond),
                        RotationsPerSecondPerSecond.of(1).in(DegreesPerSecondPerSecond)));
        private static final Distance LAUNCHER_TANGENT_OFFSET = Inches.of(11.3 * Math.cos(Degrees.of(45).in(Radians)));
        private static final PIDConstants DEFAULT_TRANSLATION_PID = new PIDConstants(10);
        private static final PIDConstants DEFAULT_ROTATION_PID = new PIDConstants(7);
        private static final Angle ALLIANCE_BLUE_SIDE = Degrees.of(0.0);
        private static final Angle ALLIANCE_RED_SIDE = Degrees.of(180.0);
        private final String limelightName;
        private final RectanglePoseArea field;

        private final RobotConfig robotConfiguration;
        private final SwerveRequest.ApplyRobotSpeeds pathPlannerSwerveRequest = new SwerveRequest.ApplyRobotSpeeds();
        private final FieldCentric fieldCentricSwerveRequest = new FieldCentric();
        private final RobotCentric robotCentricSwerveRequest = new RobotCentric();
        private final FieldCentricFacingAngle fieldCentricFacingAngleSwerveRequest = new FieldCentricFacingAngle();

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
                                Inches.of(8).in(Meters),
                                Inches.of(12).in(Meters),
                                Inches.of(20.5).in(Meters),
                                Degrees.zero().in(Degrees),
                                Degrees.of(20).in(Degrees),
                                Degrees.of(-90).in(Degrees));
                LimelightHelpers.SetRobotOrientation(limelightName,
                                getPigeon2().getYaw().getValue().in(Degrees),
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0);

                fieldCentricFacingAngleSwerveRequest.HeadingController.setTolerance(Degrees.of(5).in(Radians));
                fieldCentricFacingAngleSwerveRequest.withHeadingPID(FACING_ANGLE_PID.kP, FACING_ANGLE_PID.kI,
                                FACING_ANGLE_PID.kD);

                alignController.setTolerance(Degrees.of(5).in(Degrees));
                alignController.enableContinuousInput(Degrees.of(-180).in(Degrees), Degrees.of(180).in(Degrees));

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

                SmartDashboard.putData("Facing Angle PID", new Sendable() {
                        @Override
                        public void initSendable(SendableBuilder builder) {
                                builder.setSmartDashboardType("PIDController");
                                builder.addDoubleProperty("p", () -> getP(), (input) -> setP(input));
                                builder.addDoubleProperty("i", () -> getI(), (input) -> setI(input));
                                builder.addDoubleProperty("d", () -> getD(), (input) -> setD(input));
                        }
                });

                SmartDashboard.putData("Align Controller", alignController);
        }

        private double getP() {
                return fieldCentricFacingAngleSwerveRequest.HeadingController.getP();
        }

        private double getI() {
                return fieldCentricFacingAngleSwerveRequest.HeadingController.getI();
        }

        private double getD() {
                return fieldCentricFacingAngleSwerveRequest.HeadingController.getD();
        }

        private void setP(double p) {
                fieldCentricFacingAngleSwerveRequest.withHeadingPID(p,
                                fieldCentricFacingAngleSwerveRequest.HeadingController.getI(),
                                fieldCentricFacingAngleSwerveRequest.HeadingController.getD());
        }

        private void setI(double i) {
                fieldCentricFacingAngleSwerveRequest.withHeadingPID(
                                fieldCentricFacingAngleSwerveRequest.HeadingController.getP(), i,
                                fieldCentricFacingAngleSwerveRequest.HeadingController.getD());
        }

        private void setD(double d) {
                fieldCentricFacingAngleSwerveRequest.withHeadingPID(
                                fieldCentricFacingAngleSwerveRequest.HeadingController.getP(),
                                fieldCentricFacingAngleSwerveRequest.HeadingController.getI(), d);
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
                        Angle targetAngle) {
                Angle robotHeading = Radians.of(MathUtil.angleModulus(getAngle().in(Radians)));

                AngularVelocity angleVelocity = RPM.zero();

                SmartDashboard.putNumber("Robot Heading Wrapped", robotHeading.in(Degrees));
                SmartDashboard.putNumber("Target Angle", targetAngle.in(Degrees));

                if (!robotHeading.isNear(targetAngle, Degrees.of(5))) {
                        double kP = 0.3;
                        angleVelocity = DegreesPerSecond.of(targetAngle.minus(robotHeading).times(kP).in(Degrees));
                }

                setControl(
                                fieldCentricSwerveRequest
                                                .withVelocityX(y)
                                                .withVelocityY(x)
                                                .withRotationalRate(angleVelocity));

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
                }).withName("Subsystems/" + getName() + "/angleToOutpost")
                                .unless(DriverStation.getAlliance()::isEmpty);
        }

        private Translation2d getHubPosition(Alliance alliance) {
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

                return new Translation2d(xHub, yHub);
        }

        public Command angleToHub(
                        Supplier<Dimensionless> x,
                        Supplier<Dimensionless> y) {
                return run(() -> {
                        Optional<Alliance> maybeAlliance = DriverStation.getAlliance();

                        maybeAlliance.ifPresent(alliance -> {
                                Translation2d robotPosition = getState().Pose.getTranslation();

                                Translation2d hubPosition = getHubPosition(alliance);

                                Angle launcherOffset = Radians.of(Math.asin(
                                                LAUNCHER_TANGENT_OFFSET.div(Meters.of(
                                                                robotPosition.getDistance(hubPosition)))
                                                                .in(Value)));

                                Translation2d targetTranslation = robotPosition.minus(hubPosition);

                                Angle targetAngle = Radians
                                                .of(Math.atan2(targetTranslation.getY(), targetTranslation.getX()))
                                                .plus(Degrees.of(90)).minus(launcherOffset);

                                Angle wrapTargetAngle = Radians.of(MathUtil.angleModulus(targetAngle.in(Radians)));

                                moveWithLockedAngle(
                                                percentageToLinearVelocity(MAX_LINEAR_VELOCITY, x),
                                                percentageToLinearVelocity(MAX_LINEAR_VELOCITY, y),
                                                wrapTargetAngle);
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
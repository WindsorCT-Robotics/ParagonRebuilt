package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Value;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.launch_calculator.ShotCalculator;
import frc.robot.generated.launch_calculator.ShotCalculator.Config;
import frc.robot.generated.launch_calculator.ShotCalculator.LaunchParameters;
import frc.robot.subsystems.BayDoor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Drive.RelativeReference;
import frc.robot.utils.AngleUtil;
import frc.robot.utils.ControllerUtil;

public class RobotContainer implements Sendable {
        private static final Dimensionless DRIVER_CONTROLLER_DEADBAND = Percent.of(5);
        private static final double MOVE_CURVE = 2.0;
        private static final double TURN_CURVE = 2.0;

        // Controller Triggers
        private final SendableTrigger t_autoScore;
        private final SendableTrigger t_manualScore;
        private final SendableTrigger t_partialManualScore;
        private final SendableTrigger t_autoSnowBlow;
        private final SendableTrigger t_unjam;
        private final SendableTrigger t_incrementLauncherOffset;
        private final SendableTrigger t_decrementLauncherOffset;
        private final SendableTrigger t_faceRedAlliance;
        private final ToggleableTrigger t_autoIntake;
        private final SendableTrigger t_autoShuttle;
        private final SendableTrigger t_openBayDoor;
        private final SendableTrigger t_closeBayDoor;
        private final SendableTrigger t_switchRelativeReference;
        private final SendableTrigger t_resetGyro;
        private final SendableTrigger t_climb;
        private final SendableTrigger t_climb_home;

        // Conditional Triggers
        private final SendableTrigger t_hubLaunchValid;
        private final SendableTrigger t_snowBlowValid;
        private final SendableTrigger t_attemptToScore;
        private final SendableTrigger t_onAllianceSide;
        private final SendableTrigger t_prepareFuel;

        // Command Triggers
        private final SendableTrigger cmd_autoScore_launchFuel;
        private final SendableTrigger cmd_autoScore_indexFuel;

        private final Drive drive;
        private final Spindexer spindexer;
        private final BayDoor bayDoor;
        private final Intake intake;
        private final Kicker kicker;
        private final Launcher launcher;
        // private final Climber climber;

        private final CommandXboxController driver = new CommandXboxController(0);
        private final CommandXboxController operator = new CommandXboxController(1);

        private final Supplier<Dimensionless> moveX = () -> ControllerUtil
                        .getAxisWithDeadBandAndCurve(driver.getLeftX(), DRIVER_CONTROLLER_DEADBAND, MOVE_CURVE);
        private final Supplier<Dimensionless> moveY = () -> ControllerUtil
                        .getAxisWithDeadBandAndCurve(driver.getLeftY(), DRIVER_CONTROLLER_DEADBAND, MOVE_CURVE);
        private final Supplier<Dimensionless> turnX = () -> ControllerUtil
                        .getAxisWithDeadBandAndCurve(driver.getRightX(), DRIVER_CONTROLLER_DEADBAND, TURN_CURVE);

        private final ShotCalculator launchCalculator;
        private final ShotCalculator.Config launcherCalculatorConfig = new Config();

        private final Supplier<Optional<ShotCalculator.LaunchParameters>> hubLaunchSupplier;
        private final Supplier<Optional<ShotCalculator.LaunchParameters>> snowBlowLaunchSupplier;

        private Optional<LaunchParameters> hubLaunchParameters = Optional.empty();
        private Optional<LaunchParameters> snowBlowLaunchParameters = Optional.empty();

        private final SendableChooser<Command> autoChooser;

        private RelativeReference relativeReference;

        public RobotContainer() {
                try {
                        drive = new Drive(Drive.class.getSimpleName());
                } catch (IOException | ParseException e) {
                        throw new IllegalStateException("PathPlanner Configuration failed to load.", e);
                }

                intake = new Intake(Intake.class.getSimpleName());
                bayDoor = new BayDoor(BayDoor.class.getSimpleName());
                spindexer = new Spindexer(Spindexer.class.getSimpleName());
                launcher = new Launcher(Launcher.class.getSimpleName());
                kicker = new Kicker(Kicker.class.getSimpleName());
                // climber = new Climber(Climber.class.getSimpleName());

                relativeReference = RelativeReference.FIELD_CENTRIC;

                launcherCalculatorConfig.launcherOffsetX = Inches.of(-7).in(Meters);
                launcherCalculatorConfig.launcherOffsetY = Inches.of(-9).in(Meters);
                launcherCalculatorConfig.shooterAngleOffsetRad = Math.PI / 2;
                launcherCalculatorConfig.phaseDelayMs = 30.0;
                launcherCalculatorConfig.mechLatencyMs = 20.0;
                launcherCalculatorConfig.maxTiltDeg = 5.0;
                launcherCalculatorConfig.headingSpeedScalar = 1;
                launcherCalculatorConfig.headingReferenceDistance = 2.5;

                launchCalculator = new ShotCalculator(launcherCalculatorConfig);
                launchCalculator.loadLUTEntry(1.5, 2000, 1);
                launchCalculator.loadLUTEntry(2.0, 2125, 1);
                launchCalculator.loadLUTEntry(2.5, 2225, 1);
                launchCalculator.loadLUTEntry(3, 2325, 1);
                launchCalculator.loadLUTEntry(3.5, 2450, 1);
                launchCalculator.loadLUTEntry(4, 2590, 1);
                launchCalculator.loadLUTEntry(4.5, 2690, 1);
                launchCalculator.loadLUTEntry(5, 2850, 1);

                hubLaunchSupplier = () -> DriverStation.getAlliance()
                                .map(alliance -> getLaunchParametersFor(drive.getHubTarget(alliance)));

                snowBlowLaunchSupplier = () -> DriverStation.getAlliance()
                                .map(alliance -> getLaunchParametersFor(drive.getSnowblowTarget(alliance)));

                // Controller Triggers
                t_autoScore               = new SendableTrigger(driver.rightBumper(), "c_autoScore");
                t_manualScore             = new SendableTrigger(operator.b(), "c_manualScore");
                t_partialManualScore      = new SendableTrigger(operator.povLeft(), "c_partialManualScore");
                t_autoSnowBlow            = new SendableTrigger(driver.rightTrigger(Percent.of(0.2).in(Value)), "c_autoSnowBlow");
                t_unjam                   = new SendableTrigger(operator.a(), "c_unjam");
                t_incrementLauncherOffset = new SendableTrigger(operator.rightBumper(), "c_incrementLauncherOffset");
                t_decrementLauncherOffset = new SendableTrigger(operator.leftBumper(), "c_decrementLauncherOffset");
                t_faceRedAlliance         = new SendableTrigger(driver.leftStick(), "c_faceRedAlliance");
                t_autoIntake              = new ToggleableTrigger(driver.x());
                t_autoShuttle             = new SendableTrigger(driver.b(), "c_autoShuttle");
                t_openBayDoor             = new SendableTrigger(operator.povDown(), "c_openBayDoor");
                t_closeBayDoor            = new SendableTrigger(operator.povUp(), "c_closeBayDoor");
                t_switchRelativeReference = new SendableTrigger(driver.leftBumper(), "c_switchRelativeReference");
                t_resetGyro               = new SendableTrigger(driver.povDown(), "c_resetGyro");
                t_climb                   = new SendableTrigger(operator.rightTrigger(), "c_climb");
                t_climb_home              = new SendableTrigger(operator.start().and(operator.back()), "c_climb_home");

                // Conditional Trigger
                t_hubLaunchValid = new SendableTrigger(
                                () -> hubLaunchSupplier.get().map(parameters -> parameters.isValid())
                                                .orElse(false),
                                "nc_hubLaunchValid");

                t_snowBlowValid = new SendableTrigger(
                                () -> snowBlowLaunchSupplier.get().map(parameters -> parameters.isValid())
                                                .orElse(false),
                                "nc_snowBlowValid");

                t_attemptToScore = new SendableTrigger(t_autoScore.or(t_partialManualScore).or(t_manualScore),
                                "nc_attemptToScore");
                t_onAllianceSide = new SendableTrigger(drive.onAllianceSide.and(() -> DriverStation.isTeleop()),
                                "nc_onAllianceSide");

                t_prepareFuel = new SendableTrigger(t_onAllianceSide.and(t_attemptToScore.negate()), "nc_prepareFuel");

                // Command Trigger
                cmd_autoScore_launchFuel = new SendableTrigger(t_autoScore.and(t_onAllianceSide), "cmd_autoScore_launchFuel");
                cmd_autoScore_indexFuel = new SendableTrigger(t_autoScore.and(t_hubLaunchValid).and(t_onAllianceSide).and(t_unjam.negate()), "cmd_autoScore_indexFuel");

                autoChooser = new SendableChooser<>();
                initPathPlannerCommands();
                initSmartdashBoard();

                bindCommands();
        }

        private void initSmartdashBoard() {
                SmartDashboard.putData(drive);
                SmartDashboard.putData(intake);
                SmartDashboard.putData(bayDoor);
                SmartDashboard.putData(spindexer);
                SmartDashboard.putData(launcher);
                SmartDashboard.putData(kicker);
                SmartDashboard.putData(CommandScheduler.getInstance());

                SmartDashboard.putData("Robot Container", this);
                SmartDashboard.putData("Robot Container/Controllers/Driver", driver.getHID());
                SmartDashboard.putData("Robot Container/Controllers/Operator", operator.getHID());
                SmartDashboard.putData("Robot Container/Autonomous", autoChooser);
                SmartDashboard.putData("Robot Container/Launch Calculator", launchCalculator);
                SmartDashboard.putData("Robot Container/Triggers", t_autoScore);
                SmartDashboard.putData("Robot Container/Triggers", t_manualScore);
                SmartDashboard.putData("Robot Container/Triggers", t_partialManualScore);
                SmartDashboard.putData("Robot Container/Triggers", t_autoSnowBlow);
                SmartDashboard.putData("Robot Container/Triggers", t_unjam);
                SmartDashboard.putData("Robot Container/Triggers", t_incrementLauncherOffset);
                SmartDashboard.putData("Robot Container/Triggers", t_decrementLauncherOffset);
                SmartDashboard.putData("Robot Container/Triggers", t_faceRedAlliance);
                SmartDashboard.putData("Robot Container/Triggers", t_autoShuttle);
                SmartDashboard.putData("Robot Container/Triggers", t_openBayDoor);
                SmartDashboard.putData("Robot Container/Triggers", t_closeBayDoor);
                SmartDashboard.putData("Robot Container/Triggers", t_switchRelativeReference);
                SmartDashboard.putData("Robot Container/Triggers", t_resetGyro);
                SmartDashboard.putData("Robot Container/Triggers", t_climb);
                SmartDashboard.putData("Robot Container/Triggers", t_climb_home);
                SmartDashboard.putData("Robot Container/Triggers", t_hubLaunchValid);
                SmartDashboard.putData("Robot Container/Triggers", t_snowBlowValid);
                SmartDashboard.putData("Robot Container/Triggers", t_attemptToScore);
                SmartDashboard.putData("Robot Container/Triggers", t_onAllianceSide);
        }

        @Override
        public void initSendable(SendableBuilder builder) {

        }

        private LaunchParameters getLaunchParametersFor(Translation2d target) {
                SwerveDriveState driveState = drive.getState();
                Pigeon2 gyro = drive.getPigeon2();

                Pose2d drivePosition = drive.getRawPosition();
                Angle  pitch         = AngleUtil.wrap(gyro.getPitch().getValue());
                Angle  yaw           = AngleUtil.wrap(gyro.getYaw().getValue());
                Angle  roll          = AngleUtil.wrap(gyro.getRoll().getValue());

                ChassisSpeeds robotSpeeds = driveState.Speeds;
                ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds,
                                new Rotation2d(yaw));

                ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
                                drivePosition,
                                fieldSpeeds,
                                robotSpeeds,
                                target,
                                0.9,
                                pitch.in(Degrees),
                                roll.in(Degrees));

                return launchCalculator.calculate(inputs);
        }

        public void updateLaunchParameters() {
                hubLaunchParameters = hubLaunchSupplier.get();
                snowBlowLaunchParameters = snowBlowLaunchSupplier.get();
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        private void initPathPlannerCommands() {
                // NamedCommands.registerCommand("score",
                // angleToHub()
                // .alongWith(launcher.launchFuel(null))
                // .alongWith(kicker.kickFuel(null)
                // .alongWith(spindexer.indexFuel())
                // .alongWith(bayDoor.agitateHighFuel())));

                NamedCommands.registerCommand("baydooropen", bayDoor.open());
                NamedCommands.registerCommand("baydoorclose", bayDoor.close());
                NamedCommands.registerCommand("intakefuel", intake.intakeFuel());
        }

        private void switchRelativeReference() {
                if (relativeReference == RelativeReference.FIELD_CENTRIC) {
                        relativeReference = RelativeReference.ROBOT_CENTRIC;
                } else {
                        relativeReference = RelativeReference.FIELD_CENTRIC;
                }
        }

        private RelativeReference getRelativeReference() {
                return relativeReference;
        }

        private Command angleToRedAlliance() {
                return drive.angleToRedAlliance(moveX, moveY);
        }

        private AngularVelocity launchVelocityToHub() {
                return hubLaunchParameters.map(parameters -> RPM.of(parameters.rpm())).orElse(RPM.zero());
        }

        private Optional<Angle> angleToHub() {
                return hubLaunchParameters.map(parameters -> parameters.driveAngle().getMeasure());
        }

        private Optional<AngularVelocity> angleToHubWithFF() {
                return hubLaunchParameters
                                .map(parameters -> RadiansPerSecond.of(parameters.driveAngularVelocityRadPerSec()));
        }

        private AngularVelocity launchVelocityToSnowBlow() {
                return snowBlowLaunchParameters.map(parameters -> RPM.of(parameters.rpm())).orElse(RPM.zero());
        }

        private Optional<Angle> angleToSnowBlow() {
                return snowBlowLaunchParameters.map(parameters -> parameters.driveAngle().getMeasure());
        }

        private Optional<AngularVelocity> angleToSnowBlowWithFF() {
                return snowBlowLaunchParameters
                                .map(parameters -> RadiansPerSecond.of(parameters.driveAngularVelocityRadPerSec()));
        }

        private void bindCommands() {
                drive.setDefaultCommand(
                                drive.moveWithPercentages(moveX, moveY, turnX, this::getRelativeReference)
                                                .withName("Drive With Percentages"));

                intake.setDefaultCommand(intake.stopIntake().withName("Stop Intake"));

                // climber.setDefaultCommand(climber.home().withName("Home Climber"));

                t_switchRelativeReference.onTrue(new InstantCommand(() -> switchRelativeReference()));

                t_prepareFuel.whileTrue(launcher.prepareFuel().withName("Launcher Prepare Fuel"));
                t_prepareFuel.whileTrue(kicker.prepareFuel().withName("Kicker Prepare Fuel"));
                t_prepareFuel.whileTrue(spindexer.prepareFuel().withName("Spindexer Prepare Fuel"));

                bindAutoScore();
                bindSnowBlow();
                bindPartialManualScore();
                bindManualScore();

                t_unjam.whileTrue(bayDoor.open().alongWith(spindexer.agitateFuel()));

                t_autoIntake.getTrigger().whileTrue(
                                bayDoor.open().alongWith(intake.intakeFuel())
                                                .until(t_attemptToScore)
                                                .unless(t_attemptToScore)
                                                .repeatedly());

                t_autoShuttle.onTrue(
                                bayDoor.open().alongWith(intake.shuttleFuel())
                                                .until(t_attemptToScore)
                                                .unless(t_attemptToScore)
                                                .repeatedly());
                t_openBayDoor.onTrue(bayDoor.open());
                t_closeBayDoor.onTrue(bayDoor.close());

                t_faceRedAlliance.whileTrue(angleToRedAlliance());

                // t_climb.whileTrue(bayDoor.close().andThen(climber.open())
                // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
                // t_climb.whileFalse(climber.close().unless(() ->
                // DriverStation.isAutonomous()));
                // t_climb_home.onTrue(climber.home());

                t_incrementLauncherOffset
                                .onTrue(new InstantCommand(() -> launchCalculator.adjustOffset(RPM.of(25).in(RPM))));
                t_decrementLauncherOffset
                                .onTrue(new InstantCommand(() -> launchCalculator.adjustOffset(RPM.of(-25).in(RPM))));

                t_resetGyro.onTrue(drive.resetGyroCommand());
        }

        private void bindAutoScore() {
                cmd_autoScore_launchFuel.whileTrue(launcher.launchFuel(() -> launchVelocityToHub())
                .withName("Launch Fuel To Hub"));
                cmd_autoScore_launchFuel.whileTrue(kicker.kickFuel(() -> launchVelocityToHub())
                .withName("Kick Fuel To Hub"));
                cmd_autoScore_indexFuel.whileTrue(spindexer.indexFuel().withName("Index Fuel To Hub"));
                t_autoScore.whileTrue(drive.aimToWithFF(moveX, moveY, () -> angleToHub(), () -> angleToHubWithFF()).withName("Auto Aim To Hub"));
        }

        private void bindSnowBlow() {
                t_autoSnowBlow.whileTrue(launcher.launchFuel(() -> launchVelocityToSnowBlow())
                                .withName("Launch Fuel To SnowBlow"));
                t_autoSnowBlow.whileTrue(
                                kicker.kickFuel(() -> launchVelocityToSnowBlow()).withName("Kick Fuel To SnowBlow"));
                t_autoSnowBlow.whileTrue(spindexer.indexFuel().until(t_snowBlowValid.negate())
                                .withName("Index Fuel To SnowBlow"));
                t_autoSnowBlow.whileTrue(
                                drive.aimToWithFF(moveX, moveY, () -> angleToSnowBlow(), () -> angleToSnowBlowWithFF())
                                                .withName("Auto Aim To SnowBlow"));
                t_autoSnowBlow.whileTrue(bayDoor.open().withName("Open Bay Door To SnowBlow"));
                t_autoSnowBlow.whileTrue(intake.intakeFuel().withName("Intake Fuel To SnowBlow"));
        }

        private void bindPartialManualScore() {
                t_partialManualScore.whileTrue(launcher.smartDashboardLaunchFuel());
                t_partialManualScore.whileTrue(kicker.smartDashboardKickFuel());
                t_partialManualScore.whileTrue(spindexer.indexFuel());
                t_partialManualScore.whileTrue(drive.aimTo(moveX, moveY, () -> angleToHub()));
        }

        private void bindManualScore() {
                t_manualScore.whileTrue(launcher.smartDashboardLaunchFuel());
                t_manualScore.whileTrue(kicker.smartDashboardKickFuel());
                t_manualScore.whileTrue(spindexer.indexFuel());
        }
}
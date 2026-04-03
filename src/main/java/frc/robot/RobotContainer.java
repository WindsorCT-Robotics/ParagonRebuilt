package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        private final Trigger t_autoScore;
        private final Trigger t_manualScore;
        private final Trigger t_partialManualScore;
        private final Trigger t_autoSnowBlow;
        private final Trigger t_unjam;
        private final Trigger t_incrementLauncherOffset;
        private final Trigger t_decrementLauncherOffset;
        private final Trigger t_faceRedAlliance;
        private final Trigger t_autoIntake;
        private final Trigger t_autoShuttle;
        private final Trigger t_homeBayDoor;
        private final Trigger t_openBayDoor;
        private final Trigger t_closeBayDoor;
        private final Trigger t_switchRelativeReference;
        private final Trigger t_resetGyro;

        // Conditional Triggers
        private final Trigger t_scoreValid;
        private final Trigger t_snowBlowValid;

        private final Drive drive;
        private final Spindexer spindexer;
        private final BayDoor bayDoor;
        private final Intake intake;
        private final Kicker kicker;
        private final Launcher launcher;

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

                relativeReference = RelativeReference.FIELD_CENTRIC;

                launcherCalculatorConfig.launcherOffsetX = Inches.of(-7).in(Meters);
                launcherCalculatorConfig.launcherOffsetY = Inches.of(-9).in(Meters);
                launcherCalculatorConfig.shooterAngleOffsetRad = Math.PI / 2;
                launcherCalculatorConfig.phaseDelayMs = 30.0;
                launcherCalculatorConfig.mechLatencyMs = 20.0;
                launcherCalculatorConfig.maxTiltDeg = 5.0;
                launcherCalculatorConfig.headingSpeedScalar = 0.0;
                launcherCalculatorConfig.headingReferenceDistance = 0.0;
                launcherCalculatorConfig.headingMaxErrorRad = Degrees.of(1).in(Radians);

                launchCalculator = new ShotCalculator(launcherCalculatorConfig);
                launchCalculator.loadLUTEntry(1.5, 2000, 0);
                launchCalculator.loadLUTEntry(2.0, 2125, 0);
                launchCalculator.loadLUTEntry(2.5, 2225, 0);
                launchCalculator.loadLUTEntry(3, 2325, 0);
                launchCalculator.loadLUTEntry(3.5, 2450, 0);
                launchCalculator.loadLUTEntry(4, 2590, 0);
                launchCalculator.loadLUTEntry(4.5, 2690, 0);
                launchCalculator.loadLUTEntry(5, 2850, 0);

                hubLaunchSupplier = () -> DriverStation.getAlliance()
                                .map(alliance -> getLaunchParametersFor(drive.getHubTarget(alliance)));

                snowBlowLaunchSupplier = () -> DriverStation.getAlliance()
                                .map(alliance -> getLaunchParametersFor(drive.getSnowblowTarget(alliance)));

                // Controller Triggers
                t_autoScore = driver.rightBumper();
                t_manualScore = operator.b();
                t_partialManualScore = operator.povLeft();
                t_autoSnowBlow = driver.rightTrigger(Percent.of(0.2).in(Value));
                t_unjam = operator.a();
                t_incrementLauncherOffset = operator.rightBumper();
                t_decrementLauncherOffset = operator.leftBumper();
                t_faceRedAlliance = driver.leftStick();
                t_autoIntake = driver.x();
                t_autoShuttle = driver.b();
                t_homeBayDoor = operator.leftStick().and(operator.rightStick());
                t_openBayDoor = operator.povDown();
                t_closeBayDoor = operator.povUp();
                t_switchRelativeReference = driver.leftBumper();
                t_resetGyro = driver.povDown();

                t_scoreValid = new Trigger(
                                () -> hubLaunchSupplier.get().map(parameters -> parameters.isValid()).orElse(false));

                t_snowBlowValid = new Trigger(() -> snowBlowLaunchSupplier.get().map(parameters -> parameters.isValid())
                                .orElse(false));

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
                SmartDashboard.putData("Robot Container/Controllers/Driver", driver.getHID());
                SmartDashboard.putData("Robot Container/Controllers/Operator", operator.getHID());
                SmartDashboard.putData("Robot Container/Autonomous", autoChooser);
                SmartDashboard.putData("Robot Container", this);
                SmartDashboard.putData(CommandScheduler.getInstance());
        }

        @Override
        public void initSendable(SendableBuilder builder) {

        }

        private LaunchParameters getLaunchParametersFor(Translation2d target) {
                SwerveDriveState driveState = drive.getState();
                Pigeon2 gyro = drive.getPigeon2();

                Pose2d drivePosition = driveState.Pose;
                Angle pitch = AngleUtil.wrap(gyro.getPitch().getValue());
                Angle yaw = AngleUtil.wrap(gyro.getYaw().getValue());
                Angle roll = AngleUtil.wrap(gyro.getRoll().getValue());

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
                NamedCommands.registerCommand("baydoorhome", bayDoor.ensuredHome());
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

        private Command prepareFuel() {
                return launcher.prepareLaunch().alongWith(kicker.prepareFuel());
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

        private AngularVelocity launchVelocityToSnowBlow() {
                return snowBlowLaunchParameters.map(parameters -> RPM.of(parameters.rpm())).orElse(RPM.zero());
        }

        private Optional<Angle> angleToSnowBlow() {
                return snowBlowLaunchParameters.map(parameters -> parameters.driveAngle().getMeasure());
        }

        private void bindCommands() {
                drive.setDefaultCommand(
                                drive.moveWithPercentages(moveX, moveY, turnX, this::getRelativeReference)
                                                .withName("Drive With Percentages"));

                bayDoor.setDefaultCommand(bayDoor.ensuredHome().withName("Home Baydoor"));

                intake.setDefaultCommand(intake.stopIntake().withName("Stop Intake"));

                t_switchRelativeReference.onTrue(new InstantCommand(() -> switchRelativeReference()));

                // TODO: change this somehow.
                drive.onAllianceSide.and(() -> DriverStation.isTeleop()).whileTrue(prepareFuel()
                                .until(t_autoScore.or(t_autoSnowBlow).or(t_manualScore).or(t_partialManualScore)));

                t_autoScore.whileTrue(launcher.launchFuel(() -> launchVelocityToHub()));
                t_autoScore.whileTrue(kicker.kickFuel(() -> launchVelocityToHub()));
                t_autoScore.whileTrue(spindexer.indexFuel().until(t_scoreValid.negate()));
                t_autoScore.whileTrue(drive.aimTo(moveX, moveY, () -> angleToHub()));

                t_autoSnowBlow.whileTrue(launcher.launchFuel(() -> launchVelocityToSnowBlow()));
                t_autoSnowBlow.whileTrue(kicker.kickFuel(() -> launchVelocityToSnowBlow()));
                t_autoSnowBlow.whileTrue(spindexer.indexFuel().until(t_snowBlowValid.negate()));
                t_autoSnowBlow.whileTrue(drive.aimTo(moveX, moveY, () -> angleToSnowBlow()));
                t_autoSnowBlow.whileTrue(bayDoor.open());
                t_autoSnowBlow.whileTrue(intake.intakeFuel());

                t_partialManualScore.whileTrue(launcher.smartDashboardLaunchFuel());
                t_partialManualScore.whileTrue(kicker.smartDashboardKickFuel());
                t_partialManualScore.whileTrue(spindexer.indexFuel());
                t_partialManualScore.whileTrue(drive.aimTo(moveX, moveY, () -> angleToHub()));

                t_manualScore.whileTrue(launcher.smartDashboardLaunchFuel());
                t_manualScore.whileTrue(kicker.smartDashboardKickFuel());
                t_manualScore.whileTrue(spindexer.indexFuel());

                t_unjam.whileTrue(bayDoor.open().alongWith(spindexer.agitateFuel()));

                t_autoIntake.onTrue(bayDoor.open().alongWith(intake.intakeFuel()));
                t_autoShuttle.onTrue(bayDoor.open().alongWith(intake.shuttleFuel()));
                t_openBayDoor.onTrue(bayDoor.open());
                t_closeBayDoor.onTrue(bayDoor.close());
                t_homeBayDoor.onTrue(bayDoor.ensuredHome());

                t_faceRedAlliance.whileTrue(angleToRedAlliance());

                t_incrementLauncherOffset
                                .onTrue(new InstantCommand(() -> launchCalculator.adjustOffset(RPM.of(25).in(RPM))));
                t_decrementLauncherOffset
                                .onTrue(new InstantCommand(() -> launchCalculator.adjustOffset(RPM.of(-25).in(RPM))));

                t_resetGyro.onTrue(drive.resetGyroCommand());
        }
}
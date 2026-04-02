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

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoScore;
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

        // Command Triggers
        // private final Trigger t_autoScore_aligned;
        // private final Trigger t_autoScore_notAligned;
        // private final Trigger t_autoSnowBlow_aligned;
        // private final Trigger t_autoSnowBlow_notAligned;
        // private final Trigger t_partialManualScore_aligned;
        // private final Trigger t_partialManualScore_notAligned;

        private final Drive drive;
        private final Spindexer spindexer;
        private final BayDoor bayDoor;
        private final Intake intake;
        private final Kicker kicker;
        private final Launcher launcher;

        private final CommandXboxController driver = new CommandXboxController(0);
        private final CommandXboxController operator = new CommandXboxController(1);

        private final ShotCalculator launchCalculator;
        private final ShotCalculator.Config launcherCalculatorConfig = new Config();

        private final Supplier<Optional<ShotCalculator.LaunchParameters>> hubLaunchSupplier;
        private final Supplier<Optional<ShotCalculator.LaunchParameters>> snowBlowSupplier;

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

                hubLaunchSupplier = () -> {
                        Optional<Alliance> alliance = DriverStation.getAlliance();

                        if (alliance.isEmpty()) {
                                return Optional.empty();
                        }

                        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
                                        drive.getState().Pose,
                                        ChassisSpeeds.fromFieldRelativeSpeeds(drive.getState().Speeds,
                                                        new Rotation2d(drive.getAngle())),
                                        drive.getState().Speeds,
                                        drive.getHubTarget(alliance.get()),
                                        drive.getHubDirection(alliance.get()),
                                        0.9,
                                        drive.getPigeon2().getPitch().getValue().in(Degrees),
                                        drive.getPigeon2().getRoll().getValue().in(Degrees));

                        LaunchParameters launchParameters = launchCalculator.calculate(inputs);
                        return Optional.of(launchParameters);
                };

                // TODO: Do something with the getHubDirection
                snowBlowSupplier = () -> {
                        Optional<Alliance> alliance = DriverStation.getAlliance();

                        if (alliance.isEmpty()) {
                                return Optional.empty();
                        }

                        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
                                        drive.getState().Pose,
                                        ChassisSpeeds.fromFieldRelativeSpeeds(drive.getState().Speeds,
                                                        new Rotation2d(drive.getAngle())),
                                        drive.getState().Speeds,
                                        drive.getHubTarget(alliance.get()),
                                        drive.getHubDirection(alliance.get()),
                                        0.9,
                                        drive.getPigeon2().getPitch().getValue().in(Degrees),
                                        drive.getPigeon2().getPitch().getValue().in(Degrees));

                        LaunchParameters launchParameters = launchCalculator.calculate(inputs);
                        return Optional.of(launchParameters);
                };

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

                // Command Triggers
                // t_autoSnowBlow_aligned = t_autoSnowBlow.and(drive.onAllianceSide.negate())
                // .and(drive.launcherAlignedToSnowblow);
                // t_autoSnowBlow_notAligned = t_autoSnowBlow.and(drive.onAllianceSide.negate())
                // .and(drive.launcherAlignedToSnowblow.negate());

                // t_partialManualScore_aligned =
                // t_partialManualScore.and(drive.launcherAlignedToHub);
                // t_partialManualScore_notAligned =
                // t_partialManualScore.and(drive.launcherAlignedToHub.negate());

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

        // private Command angleToHub() {
        // return drive.angleToHub(
        // () -> ControllerUtil.getAxisWithDeadBandAndCurve(driver.getLeftX(),
        // DRIVER_CONTROLLER_DEADBAND,
        // MOVE_CURVE),
        // () -> ControllerUtil.getAxisWithDeadBandAndCurve(driver.getLeftY(),
        // DRIVER_CONTROLLER_DEADBAND,
        // MOVE_CURVE));
        // }

        // private Command angleToSnowBlow() {
        // return drive.angleToSnowblow(
        // () -> ControllerUtil.getAxisWithDeadBandAndCurve(driver.getLeftX(),
        // DRIVER_CONTROLLER_DEADBAND,
        // MOVE_CURVE),
        // () -> ControllerUtil.getAxisWithDeadBandAndCurve(driver.getLeftY(),
        // DRIVER_CONTROLLER_DEADBAND,
        // MOVE_CURVE));
        // }

        private Command angleToRedAlliance() {
                return drive.angleToRedAlliance(
                                () -> ControllerUtil.getAxisWithDeadBandAndCurve(driver.getLeftX(),
                                                DRIVER_CONTROLLER_DEADBAND,
                                                MOVE_CURVE),
                                () -> ControllerUtil.getAxisWithDeadBandAndCurve(driver.getLeftY(),
                                                DRIVER_CONTROLLER_DEADBAND,
                                                MOVE_CURVE));
        }

        private void bindCommands() {
                drive.setDefaultCommand(
                                drive.moveWithPercentages(
                                                () -> ControllerUtil.getAxisWithDeadBandAndCurve(driver.getLeftX(),
                                                                DRIVER_CONTROLLER_DEADBAND,
                                                                MOVE_CURVE),
                                                () -> ControllerUtil.getAxisWithDeadBandAndCurve(driver.getLeftY(),
                                                                DRIVER_CONTROLLER_DEADBAND,
                                                                MOVE_CURVE),
                                                () -> ControllerUtil.getAxisWithDeadBandAndCurve(driver.getRightX(),
                                                                DRIVER_CONTROLLER_DEADBAND,
                                                                TURN_CURVE),
                                                this::getRelativeReference)
                                                .withName("Drive With Percentages"));

                bayDoor.setDefaultCommand(bayDoor.ensuredHome().withName("Home Baydoor"));

                intake.setDefaultCommand(intake.stopIntake().withName("Stop Intake"));

                t_switchRelativeReference.onTrue(new InstantCommand(() -> switchRelativeReference()));

                drive.onAllianceSide.and(() -> DriverStation.isTeleop()).whileTrue(prepareFuel()
                                .until(t_autoScore.or(t_autoSnowBlow).or(t_manualScore).or(t_partialManualScore)));

                t_autoScore.whileTrue(new AutoScore(
                                drive,
                                launcher,
                                kicker,
                                spindexer,
                                bayDoor,
                                intake,
                                hubLaunchSupplier,
                                () -> ControllerUtil.getAxisWithDeadBandAndCurve(
                                                driver.getLeftX(),
                                                DRIVER_CONTROLLER_DEADBAND,
                                                MOVE_CURVE),
                                () -> ControllerUtil.getAxisWithDeadBandAndCurve(
                                                driver.getLeftY(),
                                                DRIVER_CONTROLLER_DEADBAND,
                                                MOVE_CURVE)));

                // t_autoScore_aligned.whileTrue(
                // angleToHub()
                // .alongWith(launcher.launchFuel(null))
                // .alongWith(kicker.kickFuel(null))
                // .alongWith(spindexer.indexFuel())
                // .alongWith(bayDoor.agitateLowFuel())
                // .alongWith(intake.agitateFuel()));

                // t_autoScore_notAligned.whileTrue(
                // angleToHub()
                // .alongWith(launcher.launchFuel(null))
                // .alongWith(kicker.kickFuel(null)));

                // t_autoSnowBlow_aligned.whileTrue(
                // angleToSnowBlow()
                // .alongWith(launcher.launchFuel(null))
                // .alongWith(kicker.kickFuel(null))
                // .alongWith(spindexer.indexFuel())
                // .alongWith(bayDoor.open())
                // .alongWith(intake.intakeFuel()));

                // t_autoSnowBlow_notAligned.whileTrue(
                // angleToSnowBlow()
                // .alongWith(launcher.launchFuel(null))
                // .alongWith(kicker.kickFuel(null))
                // .alongWith(bayDoor.open())
                // .alongWith(intake.intakeFuel()));

                // t_partialManualScore_aligned.whileTrue(
                // angleToHub()
                // .alongWith(launcher.smartDashboardLaunchFuel())
                // .alongWith(kicker.smartDashboardKickFuel())
                // .alongWith(spindexer.indexFuel()));

                // t_partialManualScore_notAligned.whileTrue(
                // angleToHub()
                // .alongWith(launcher.smartDashboardLaunchFuel())
                // .alongWith(kicker.smartDashboardKickFuel()));

                t_manualScore.whileTrue(
                                launcher.smartDashboardLaunchFuel()
                                                .alongWith(kicker.smartDashboardKickFuel())
                                                .alongWith(spindexer.indexFuel()));

                t_autoIntake.onTrue(bayDoor.open().alongWith(intake.intakeFuel()));
                t_autoShuttle.onTrue(bayDoor.open().alongWith(intake.shuttleFuel()));
                t_openBayDoor.onTrue(bayDoor.open());
                t_closeBayDoor.onTrue(bayDoor.close());
                t_homeBayDoor.onTrue(bayDoor.ensuredHome());

                t_unjam.whileTrue(bayDoor.open().alongWith(spindexer.agitateFuel()));

                t_faceRedAlliance.whileTrue(angleToRedAlliance());

                t_incrementLauncherOffset.onTrue(launcher.incrementLauncherOffset());
                t_decrementLauncherOffset.onTrue(launcher.decrementLauncherOffset());

                t_resetGyro.onTrue(drive.resetGyroCommand());
        }
}
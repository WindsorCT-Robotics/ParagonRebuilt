package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.launch_calculator.ShotCalculator.LaunchParameters;
import frc.robot.subsystems.BayDoor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Spindexer;

public class Bindings {
        private static final String NAME = "Trigger";

        // Controller Triggers
        public final SendableTrigger t_autoScore;
        public final SendableTrigger t_manualScore;
        public final SendableTrigger t_partialManualScore;
        public final SendableTrigger t_autoSnowBlow;
        public final SendableTrigger t_autoUnjam;
        public final SendableTrigger t_manualUnjam;
        public final SendableTrigger t_incrementLauncherOffset;
        public final SendableTrigger t_decrementLauncherOffset;
        public final SendableTrigger t_faceRedAlliance;
        public final ToggleableTrigger t_autoIntake;
        public final SendableTrigger t_autoShuttle;
        public final SendableTrigger t_openBayDoor;
        public final SendableTrigger t_closeBayDoor;
        public final SendableTrigger t_switchRelativeReference;
        public final SendableTrigger t_resetGyro;

        // Conditional Triggers
        public final SendableTrigger t_hubLaunchValid;
        public final SendableTrigger t_snowBlowValid;
        public final SendableTrigger t_attemptToScore;
        public final SendableTrigger t_onAllianceSide;
        public final SendableTrigger t_prepareFuel;
        public final SendableTrigger t_unjam;

        // Command Triggers
        public final SendableTrigger cmd_autoScore_launchFuel;
        public final SendableTrigger cmd_autoScore_indexFuel;
        public final SendableTrigger cmd_snowBlow_launchFuel;
        public final SendableTrigger cmd_snowBlow_indexFuel;
        public final SendableTrigger cmd_partialManualScore_launchFuel;
        public final SendableTrigger cmd_partialManualScore_indexFuel;
        public final SendableTrigger cmd_manualScore_launchFuel;
        public final SendableTrigger cmd_manualScore_indexFuel;
        public final SendableTrigger cmd_autoIntake;
        public final SendableTrigger cmd_autoShuttle;
        public final SendableTrigger cmd_autoUnjam;
        public final SendableTrigger cmd_manualUnjam;
        public final SendableTrigger cmd_switchRelativeReference;
        public final SendableTrigger cmd_prepareFuel;
        public final SendableTrigger cmd_bayDoor_open;
        public final SendableTrigger cmd_bayDoor_close;

        public Bindings(
                        Drive drive,
                        Launcher launcher,
                        Kicker kicker,
                        Spindexer spindexer,
                        BayDoor bayDoor,
                        Intake intake,
                        CommandXboxController driver,
                        CommandXboxController operator,
                        Supplier<Optional<LaunchParameters>> hubLaunchParameters,
                        Supplier<Optional<LaunchParameters>> snowBlowLaunchParameters) {

                // Controller Triggers
                t_autoScore = new SendableTrigger(driver.rightBumper(), "Controller/autoScore");
                t_manualScore = new SendableTrigger(operator.b(), "Controller/manualScore");
                t_partialManualScore = new SendableTrigger(operator.povLeft(), "Controller/partialManualScore");
                t_autoSnowBlow = new SendableTrigger(driver.rightTrigger(Percent.of(0.2).in(Value)),
                                "Controller/autoSnowBlow");
                t_manualUnjam = new SendableTrigger(operator.a(), "Controller/manualUnjam");
                t_incrementLauncherOffset = new SendableTrigger(operator.rightBumper(),
                                "Controller/incrementLauncherOffset");
                t_decrementLauncherOffset = new SendableTrigger(operator.leftBumper(),
                                "Controller/decrementLauncherOffset");
                t_faceRedAlliance = new SendableTrigger(driver.leftStick(), "Controller/faceRedAlliance");
                t_autoIntake = new ToggleableTrigger(driver.x());
                t_autoShuttle = new SendableTrigger(driver.b(), "Controller/autoShuttle");
                t_openBayDoor = new SendableTrigger(operator.povDown(), "Controller/openBayDoor");
                t_closeBayDoor = new SendableTrigger(operator.povUp(), "Controller/closeBayDoor");
                t_switchRelativeReference = new SendableTrigger(driver.leftBumper(),
                                "Controller/switchRelativeReference");
                t_resetGyro = new SendableTrigger(driver.povDown(), "Controller/resetGyro");

                // Conditional Triggers
                t_hubLaunchValid = new SendableTrigger(
                                () -> hubLaunchParameters.get().map(parameters -> parameters.isValid()).orElse(false),
                                "Conditional/hubLaunchValid");

                t_snowBlowValid = new SendableTrigger(
                                () -> snowBlowLaunchParameters.get().map(parameters -> parameters.isValid())
                                                .orElse(false),
                                "Conditional/snowBlowValid");

                t_attemptToScore = new SendableTrigger(t_autoScore.or(t_partialManualScore).or(t_manualScore),
                                "Conditional/attemptToScore");
                t_onAllianceSide = new SendableTrigger(drive.onAllianceSide.and(() -> !DriverStation.isAutonomous()),
                                "Conditional/onAllianceSide");

                t_prepareFuel = new SendableTrigger(t_onAllianceSide.and(t_attemptToScore.negate()),
                                "Conditional/prepareFuel");

                t_autoUnjam = new SendableTrigger(spindexer.autoUnjamTrigger, "Conditional/autoUnjam");

                t_unjam = new SendableTrigger(() -> t_manualUnjam.getAsBoolean() && DriverStation.isTeleop(), "Conditional/unjam");

                // Command Triggers
                cmd_autoScore_launchFuel = new SendableTrigger(
                                t_autoScore
                                                .and(t_onAllianceSide),
                                "Commands/autoScore_launchFuel");
                cmd_autoScore_indexFuel = new SendableTrigger(
                                t_autoScore
                                                .and(t_hubLaunchValid)
                                                .and(t_onAllianceSide)
                                                .and(t_unjam.negate()),
                                "Commands/autoScore_indexFuel");
                cmd_snowBlow_launchFuel = new SendableTrigger(
                                t_autoSnowBlow
                                                .and(t_onAllianceSide.negate()),
                                "Commands/snowBlow_launchFuel");

                cmd_snowBlow_indexFuel = new SendableTrigger(
                                t_autoSnowBlow
                                                .and(t_onAllianceSide.negate())
                                                .and(t_snowBlowValid)
                                                .and(t_unjam.negate()),
                                "Commands/snowBlow_launchFuel");

                cmd_partialManualScore_launchFuel = new SendableTrigger(t_partialManualScore,
                                "Commands/partialManualScore_launchFuel");
                cmd_partialManualScore_indexFuel = new SendableTrigger(t_partialManualScore.and(t_unjam.negate()),
                                "Commands/partialManualScore_indexFuel");

                cmd_manualScore_launchFuel = new SendableTrigger(t_manualScore, "Commands/manualScore_launchFuel");
                cmd_manualScore_indexFuel = new SendableTrigger(t_manualScore.and(t_manualUnjam.negate()),
                                "Commands/manualScore_launchFuel");

                cmd_autoIntake = new SendableTrigger(
                                t_autoIntake.getTrigger()
                                                .and(t_autoShuttle.negate())
                                                .and(t_attemptToScore.negate())
                                                .and(t_autoSnowBlow.negate()),
                                "Commands/autoIntake");
                cmd_autoShuttle = new SendableTrigger(
                                t_autoShuttle
                                                .and(t_attemptToScore.negate()),
                                "Commands/autoIntake");
                cmd_switchRelativeReference = new SendableTrigger(driver.leftBumper(),
                                "Commands/switchRelativeReference");
                cmd_autoUnjam = new SendableTrigger(t_autoUnjam, null);
                cmd_manualUnjam = new SendableTrigger(t_manualUnjam, null);
                cmd_prepareFuel = new SendableTrigger(t_prepareFuel, "Commands/prepareFuel");
                cmd_bayDoor_open = new SendableTrigger(t_openBayDoor, "Commands/bayDoor_open");
                cmd_bayDoor_close = new SendableTrigger(t_closeBayDoor, "Commands/bayDoor_close");
        }

        public void initSmartdashBoard(String dir) {
                SmartDashboard.putData(dir + NAME, cmd_autoIntake);
                SmartDashboard.putData(dir + NAME, cmd_autoScore_indexFuel);
                SmartDashboard.putData(dir + NAME, cmd_autoScore_launchFuel);
                SmartDashboard.putData(dir + NAME, cmd_autoShuttle);
                SmartDashboard.putData(dir + NAME, cmd_autoUnjam);
                SmartDashboard.putData(dir + NAME, cmd_bayDoor_close);
                SmartDashboard.putData(dir + NAME, cmd_bayDoor_open);
                SmartDashboard.putData(dir + NAME, cmd_manualScore_indexFuel);
                SmartDashboard.putData(dir + NAME, cmd_manualScore_launchFuel);
                SmartDashboard.putData(dir + NAME, cmd_manualUnjam);
                SmartDashboard.putData(dir + NAME, cmd_partialManualScore_indexFuel);
                SmartDashboard.putData(dir + NAME, cmd_partialManualScore_launchFuel);
                SmartDashboard.putData(dir + NAME, cmd_prepareFuel);
                SmartDashboard.putData(dir + NAME, cmd_snowBlow_indexFuel);
                SmartDashboard.putData(dir + NAME, cmd_snowBlow_launchFuel);
                SmartDashboard.putData(dir + NAME, cmd_switchRelativeReference);
                SmartDashboard.putData(dir + NAME, t_attemptToScore);
                SmartDashboard.putData(dir + NAME, t_autoScore);
                SmartDashboard.putData(dir + NAME, t_autoShuttle);
                SmartDashboard.putData(dir + NAME, t_autoSnowBlow);
                SmartDashboard.putData(dir + NAME, t_autoUnjam);
                SmartDashboard.putData(dir + NAME, t_closeBayDoor);
                SmartDashboard.putData(dir + NAME, t_decrementLauncherOffset);
                SmartDashboard.putData(dir + NAME, t_faceRedAlliance);
                SmartDashboard.putData(dir + NAME, t_hubLaunchValid);
                SmartDashboard.putData(dir + NAME, t_incrementLauncherOffset);
                SmartDashboard.putData(dir + NAME, t_manualScore);
                SmartDashboard.putData(dir + NAME, t_manualUnjam);
                SmartDashboard.putData(dir + NAME, t_onAllianceSide);
                SmartDashboard.putData(dir + NAME, t_openBayDoor);
                SmartDashboard.putData(dir + NAME, t_partialManualScore);
                SmartDashboard.putData(dir + NAME, t_prepareFuel);
                SmartDashboard.putData(dir + NAME, t_resetGyro);
                SmartDashboard.putData(dir + NAME, t_snowBlowValid);
                SmartDashboard.putData(dir + NAME, t_switchRelativeReference);
                SmartDashboard.putData(dir + NAME, t_unjam);
        }
}
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
                t_autoScore = new SendableTrigger(driver.rightBumper(), "autoScore");
                t_manualScore = new SendableTrigger(operator.b(), "manualScore");
                t_partialManualScore = new SendableTrigger(operator.povLeft(), "partialManualScore");
                t_autoSnowBlow = new SendableTrigger(driver.rightTrigger(Percent.of(0.2).in(Value)),
                                "autoSnowBlow");
                t_manualUnjam = new SendableTrigger(operator.a(), "manualUnjam");
                t_incrementLauncherOffset = new SendableTrigger(operator.rightBumper(),
                                "incrementLauncherOffset");
                t_decrementLauncherOffset = new SendableTrigger(operator.leftBumper(),
                                "decrementLauncherOffset");
                t_faceRedAlliance = new SendableTrigger(driver.leftStick(), "faceRedAlliance");
                t_autoIntake = new ToggleableTrigger(driver.x());
                t_autoShuttle = new SendableTrigger(driver.b(), "autoShuttle");
                t_openBayDoor = new SendableTrigger(operator.povDown(), "openBayDoor");
                t_closeBayDoor = new SendableTrigger(operator.povUp(), "closeBayDoor");
                t_switchRelativeReference = new SendableTrigger(driver.leftBumper(),
                                "switchRelativeReference");
                t_resetGyro = new SendableTrigger(driver.povDown(), "resetGyro");

                // Conditional Triggers
                t_hubLaunchValid = new SendableTrigger(
                                () -> hubLaunchParameters.get().map(parameters -> parameters.isValid()).orElse(false),
                                "hubLaunchValid");

                t_snowBlowValid = new SendableTrigger(
                                () -> snowBlowLaunchParameters.get().map(parameters -> parameters.isValid())
                                                .orElse(false),
                                "snowBlowValid");

                t_attemptToScore = new SendableTrigger(t_autoScore.or(t_partialManualScore).or(t_manualScore),
                                "attemptToScore");
                t_onAllianceSide = new SendableTrigger(drive.onAllianceSide.and(() -> !DriverStation.isAutonomous()),
                                "onAllianceSide");

                t_prepareFuel = new SendableTrigger(t_onAllianceSide.and(t_attemptToScore.negate()),
                                "prepareFuel");

                t_autoUnjam = new SendableTrigger(spindexer.autoUnjamTrigger, "autoUnjam");

                t_unjam = new SendableTrigger(() -> t_manualUnjam.getAsBoolean() && DriverStation.isTeleop(), "unjam");

                // Command Triggers
                cmd_autoScore_launchFuel = new SendableTrigger(
                                t_autoScore
                                                .and(t_onAllianceSide),
                                "autoScore_launchFuel");
                cmd_autoScore_indexFuel = new SendableTrigger(
                                t_autoScore
                                                .and(t_hubLaunchValid)
                                                .and(t_onAllianceSide)
                                                .and(t_unjam.negate()),
                                "autoScore_indexFuel");
                cmd_snowBlow_launchFuel = new SendableTrigger(
                                t_autoSnowBlow
                                                .and(t_onAllianceSide.negate()),
                                "snowBlow_launchFuel");

                cmd_snowBlow_indexFuel = new SendableTrigger(
                                t_autoSnowBlow
                                                .and(t_onAllianceSide.negate())
                                                .and(t_snowBlowValid)
                                                .and(t_unjam.negate()),
                                "snowBlow_indexFuel");

                cmd_partialManualScore_launchFuel = new SendableTrigger(t_partialManualScore,
                                "partialManualScore_launchFuel");
                cmd_partialManualScore_indexFuel = new SendableTrigger(t_partialManualScore.and(t_unjam.negate()),
                                "partialManualScore_indexFuel");

                cmd_manualScore_launchFuel = new SendableTrigger(t_manualScore, "manualScore_launchFuel");
                cmd_manualScore_indexFuel = new SendableTrigger(t_manualScore.and(t_manualUnjam.negate()),
                                "manualScore_indexFuel");

                cmd_autoIntake = new SendableTrigger(
                                t_autoIntake.getTrigger()
                                                .and(t_autoShuttle.negate())
                                                .and(t_attemptToScore.negate())
                                                .and(t_autoSnowBlow.negate()),
                                "autoIntake");
                cmd_autoShuttle = new SendableTrigger(
                                t_autoShuttle
                                                .and(t_attemptToScore.negate()),
                                "autoShuttle");
                cmd_switchRelativeReference = new SendableTrigger(driver.leftBumper(),
                                "switchRelativeReference");
                cmd_autoUnjam = new SendableTrigger(t_autoUnjam, "autoUnjam_cmd");
                cmd_manualUnjam = new SendableTrigger(t_manualUnjam, "manualUnjam_cmd");
                cmd_prepareFuel = new SendableTrigger(t_prepareFuel, "prepareFuel");
                cmd_bayDoor_open = new SendableTrigger(t_openBayDoor, "bayDoor_open");
                cmd_bayDoor_close = new SendableTrigger(t_closeBayDoor, "bayDoor_close");
        }

        public void initSmartdashBoard(String dir) {
                String category = "Triggers/";
                SmartDashboard.putData(category, cmd_autoIntake);
                SmartDashboard.putData(category, cmd_autoScore_indexFuel);
                SmartDashboard.putData(category, cmd_autoScore_launchFuel);
                SmartDashboard.putData(category, cmd_autoShuttle);
                SmartDashboard.putData(category, cmd_autoUnjam);
                SmartDashboard.putData(category, cmd_bayDoor_close);
                SmartDashboard.putData(category, cmd_bayDoor_open);
                SmartDashboard.putData(category, cmd_manualScore_indexFuel);
                SmartDashboard.putData(category, cmd_manualScore_launchFuel);
                SmartDashboard.putData(category, cmd_manualUnjam);
                SmartDashboard.putData(category, cmd_partialManualScore_indexFuel);
                SmartDashboard.putData(category, cmd_partialManualScore_launchFuel);
                SmartDashboard.putData(category, cmd_prepareFuel);
                SmartDashboard.putData(category, cmd_snowBlow_indexFuel);
                SmartDashboard.putData(category, cmd_snowBlow_launchFuel);
                SmartDashboard.putData(category, cmd_switchRelativeReference);
                SmartDashboard.putData(category, t_attemptToScore);
                SmartDashboard.putData(category, t_autoScore);
                SmartDashboard.putData(category, t_autoShuttle);
                SmartDashboard.putData(category, t_autoSnowBlow);
                SmartDashboard.putData(category, t_autoUnjam);
                SmartDashboard.putData(category, t_closeBayDoor);
                SmartDashboard.putData(category, t_decrementLauncherOffset);
                SmartDashboard.putData(category, t_faceRedAlliance);
                SmartDashboard.putData(category, t_hubLaunchValid);
                SmartDashboard.putData(category, t_incrementLauncherOffset);
                SmartDashboard.putData(category, t_manualScore);
                SmartDashboard.putData(category, t_manualUnjam);
                SmartDashboard.putData(category, t_onAllianceSide);
                SmartDashboard.putData(category, t_openBayDoor);
                SmartDashboard.putData(category, t_partialManualScore);
                SmartDashboard.putData(category, t_prepareFuel);
                SmartDashboard.putData(category, t_resetGyro);
                SmartDashboard.putData(category, t_snowBlowValid);
                SmartDashboard.putData(category, t_switchRelativeReference);
                SmartDashboard.putData(category, t_unjam);
        }
}
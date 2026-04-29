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
        public final SendableTrigger t_towerScore;
        public final SendableTrigger t_autoSnowBlow;
        public final SendableTrigger t_manualUnjam;
        public final SendableTrigger t_incrementLauncherOffset;
        public final SendableTrigger t_decrementLauncherOffset;
        public final SendableTrigger t_faceRedAlliance;
        public final ToggleableTrigger t_autoIntake;
        public final SendableTrigger t_autoShuttle;
        public final SendableTrigger t_openBayDoor;
        public final SendableTrigger t_closeBayDoor;
        public final SendableTrigger t_bayDoorAgitation;
        public final SendableTrigger t_switchRelativeReference;
        public final SendableTrigger t_resetGyro;

        // Conditional Triggers
        public final SendableTrigger t_hubLaunchValid;
        public final SendableTrigger t_snowBlowValid;
        public final SendableTrigger t_attemptToScore;
        public final SendableTrigger t_onAllianceSide;
        public final SendableTrigger t_prepareFuel;
        public final SendableTrigger t_unjam;
        public final SendableTrigger t_nearTargetVelocity;

        // Command Triggers
        public final SendableTrigger cmd_autoScore_launchFuel;
        public final SendableTrigger cmd_autoScore_indexFuel;
        public final SendableTrigger cmd_snowBlow_launchFuel;
        public final SendableTrigger cmd_snowBlow_indexFuel;
        public final SendableTrigger cmd_partialManualScore_launchFuel;
        public final SendableTrigger cmd_partialManualScore_indexFuel;
        public final SendableTrigger cmd_towerScore_launchFuel;
        public final SendableTrigger cmd_towerScore_indexFuel;
        public final SendableTrigger cmd_manualScore_launchFuel;
        public final SendableTrigger cmd_manualScore_indexFuel;
        public final SendableTrigger cmd_autoIntake;
        public final SendableTrigger cmd_autoShuttle;
        public final SendableTrigger cmd_manualUnjam;
        public final SendableTrigger cmd_switchRelativeReference;
        public final SendableTrigger cmd_prepareFuel;
        public final SendableTrigger cmd_bayDoor_open;
        public final SendableTrigger cmd_bayDoor_close;
        public final SendableTrigger cmd_bayDoor_agitation;

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
                t_bayDoorAgitation = new SendableTrigger(operator.povRight(), "bayDoorAgitation");
                t_switchRelativeReference = new SendableTrigger(driver.leftBumper(),
                                "switchRelativeReference");
                t_resetGyro = new SendableTrigger(driver.povDown(), "resetGyro");
                t_towerScore = new SendableTrigger(driver.a(), "towerScore");

                // Conditional Triggers
                t_hubLaunchValid = new SendableTrigger(
                                () -> hubLaunchParameters.get().map(parameters -> parameters.isValid()).orElse(false),
                                "hubLaunchValid");

                t_snowBlowValid = new SendableTrigger(
                                () -> snowBlowLaunchParameters.get().map(parameters -> parameters.isValid())
                                                .orElse(false),
                                "snowBlowValid");

                t_attemptToScore = new SendableTrigger(
                                t_autoScore.or(t_partialManualScore).or(t_manualScore).or(t_towerScore),
                                "attemptToScore");
                t_onAllianceSide = new SendableTrigger(drive.onAllianceSide.and(() -> !DriverStation.isAutonomous()),
                                "onAllianceSide");

                t_prepareFuel = new SendableTrigger(t_onAllianceSide.and(t_attemptToScore.negate()),
                                "prepareFuel");

                t_unjam = new SendableTrigger(t_manualUnjam.and(() -> !DriverStation.isAutonomous()), "unjam");

                t_nearTargetVelocity = new SendableTrigger(launcher.nearTargetVelocity, "nearTargetVelocity");

                // Command Triggers
                cmd_autoScore_launchFuel = new SendableTrigger(
                                t_autoScore
                                                .and(t_onAllianceSide),
                                "autoScore_launchFuel");
                cmd_autoScore_indexFuel = new SendableTrigger(
                                t_autoScore
                                                .and(t_hubLaunchValid)
                                                .and(t_nearTargetVelocity)
                                                .and(t_onAllianceSide)
                                                .and(t_unjam.negate())
                                                .and(t_bayDoorAgitation.negate()),
                                "autoScore_indexFuel");
                cmd_snowBlow_launchFuel = new SendableTrigger(
                                t_autoSnowBlow
                                                .and(t_onAllianceSide.negate()),
                                "snowBlow_launchFuel");

                cmd_snowBlow_indexFuel = new SendableTrigger(
                                t_autoSnowBlow
                                                .and(t_onAllianceSide.negate())
                                                .and(t_snowBlowValid)
                                                .and(t_nearTargetVelocity)
                                                .and(t_unjam.negate())
                                                .and(t_bayDoorAgitation.negate()),
                                "snowBlow_indexFuel");

                cmd_partialManualScore_launchFuel = new SendableTrigger(t_partialManualScore,
                                "partialManualScore_launchFuel");
                cmd_partialManualScore_indexFuel = new SendableTrigger(t_partialManualScore
                                .and(t_nearTargetVelocity)
                                .and(t_unjam.negate())
                                .and(t_bayDoorAgitation.negate()),
                                "partialManualScore_indexFuel");

                cmd_towerScore_launchFuel = new SendableTrigger(
                                t_towerScore
                                                .and(t_unjam.negate())
                                                .and(t_bayDoorAgitation.negate()),
                                "towerScore_launchFuel");
                cmd_towerScore_indexFuel = new SendableTrigger(
                                t_towerScore
                                                .and(t_unjam.negate())
                                                .and(t_bayDoorAgitation.negate()),
                                "towerScore_indexFuel");

                cmd_manualScore_launchFuel = new SendableTrigger(t_manualScore, "manualScore_launchFuel");
                cmd_manualScore_indexFuel = new SendableTrigger(t_manualScore
                                .and(t_nearTargetVelocity)
                                .and(t_manualUnjam.negate())
                                .and(t_bayDoorAgitation.negate()),
                                "manualScore_indexFuel");

                cmd_autoIntake = new SendableTrigger(
                                t_autoIntake.getTrigger()
                                                .and(t_autoShuttle.negate())
                                                .and(t_attemptToScore.negate())
                                                .and(t_autoSnowBlow.negate())
                                                .and(t_bayDoorAgitation.negate()),
                                "autoIntake");
                cmd_autoShuttle = new SendableTrigger(
                                t_autoShuttle
                                                .and(t_attemptToScore.negate())
                                                .and(t_bayDoorAgitation.negate()),
                                "autoShuttle");
                cmd_switchRelativeReference = new SendableTrigger(driver.leftBumper(),
                                "switchRelativeReference");
                cmd_manualUnjam = new SendableTrigger(t_manualUnjam, "manualUnjam_cmd");
                cmd_prepareFuel = new SendableTrigger(t_prepareFuel, "prepareFuel");
                cmd_bayDoor_open = new SendableTrigger(t_openBayDoor.and(t_attemptToScore.negate()), "bayDoor_open");
                cmd_bayDoor_close = new SendableTrigger(t_closeBayDoor.and(t_attemptToScore.negate()), "bayDoor_close");
                cmd_bayDoor_agitation = new SendableTrigger(t_bayDoorAgitation, "bayDoor_agitation");
        }

        public void initSmartdashBoard(String dir) {
                String category = "Triggers/";
                SmartDashboard.putData(category + cmd_autoIntake.getName(), cmd_autoIntake);
                SmartDashboard.putData(category + cmd_autoScore_indexFuel.getName(), cmd_autoScore_indexFuel);
                SmartDashboard.putData(category + cmd_autoScore_launchFuel.getName(), cmd_autoScore_launchFuel);
                SmartDashboard.putData(category + cmd_autoShuttle.getName(), cmd_autoShuttle);
                SmartDashboard.putData(category + cmd_bayDoor_close.getName(), cmd_bayDoor_close);
                SmartDashboard.putData(category + cmd_bayDoor_open.getName(), cmd_bayDoor_open);
                SmartDashboard.putData(category + cmd_manualScore_indexFuel.getName(), cmd_manualScore_indexFuel);
                SmartDashboard.putData(category + cmd_manualScore_launchFuel.getName(), cmd_manualScore_launchFuel);
                SmartDashboard.putData(category + cmd_manualUnjam.getName(), cmd_manualUnjam);
                SmartDashboard.putData(category + cmd_partialManualScore_indexFuel.getName(),
                                cmd_partialManualScore_indexFuel);
                SmartDashboard.putData(category + cmd_partialManualScore_launchFuel.getName(),
                                cmd_partialManualScore_launchFuel);
                SmartDashboard.putData(category + cmd_prepareFuel.getName(), cmd_prepareFuel);
                SmartDashboard.putData(category + cmd_snowBlow_indexFuel.getName(), cmd_snowBlow_indexFuel);
                SmartDashboard.putData(category + cmd_snowBlow_launchFuel.getName(), cmd_snowBlow_launchFuel);
                SmartDashboard.putData(category + cmd_switchRelativeReference.getName(), cmd_switchRelativeReference);
                SmartDashboard.putData(category + t_attemptToScore.getName(), t_attemptToScore);
                SmartDashboard.putData(category + t_autoScore.getName(), t_autoScore);
                SmartDashboard.putData(category + t_autoShuttle.getName(), t_autoShuttle);
                SmartDashboard.putData(category + t_autoSnowBlow.getName(), t_autoSnowBlow);
                SmartDashboard.putData(category + t_closeBayDoor.getName(), t_closeBayDoor);
                SmartDashboard.putData(category + t_decrementLauncherOffset.getName(), t_decrementLauncherOffset);
                SmartDashboard.putData(category + t_faceRedAlliance.getName(), t_faceRedAlliance);
                SmartDashboard.putData(category + t_hubLaunchValid.getName(), t_hubLaunchValid);
                SmartDashboard.putData(category + t_incrementLauncherOffset.getName(), t_incrementLauncherOffset);
                SmartDashboard.putData(category + t_manualScore.getName(), t_manualScore);
                SmartDashboard.putData(category + t_manualUnjam.getName(), t_manualUnjam);
                SmartDashboard.putData(category + t_onAllianceSide.getName(), t_onAllianceSide);
                SmartDashboard.putData(category + t_openBayDoor.getName(), t_openBayDoor);
                SmartDashboard.putData(category + t_partialManualScore.getName(), t_partialManualScore);
                SmartDashboard.putData(category + t_prepareFuel.getName(), t_prepareFuel);
                SmartDashboard.putData(category + t_resetGyro.getName(), t_resetGyro);
                SmartDashboard.putData(category + t_snowBlowValid.getName(), t_snowBlowValid);
                SmartDashboard.putData(category + t_switchRelativeReference.getName(), t_switchRelativeReference);
                SmartDashboard.putData(category + t_unjam.getName(), t_unjam);
                SmartDashboard.putData(category + t_nearTargetVelocity.getName(), t_nearTargetVelocity);
        }
}
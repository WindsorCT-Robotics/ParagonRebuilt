package frc.robot.bindings;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SendableTrigger;
import frc.robot.ToggleableTrigger;
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
    public final SendableTrigger t_climb;
    public final SendableTrigger t_climb_home;

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
        t_autoSnowBlow = new SendableTrigger(driver.rightTrigger(Percent.of(0.2).in(Value)), "autoSnowBlow");
        t_manualUnjam = new SendableTrigger(operator.a(), "manualUnjam");
        t_incrementLauncherOffset = new SendableTrigger(operator.rightBumper(), "incrementLauncherOffset");
        t_decrementLauncherOffset = new SendableTrigger(operator.leftBumper(), "decrementLauncherOffset");
        t_faceRedAlliance = new SendableTrigger(driver.leftStick(), "faceRedAlliance");
        t_autoIntake = new ToggleableTrigger(driver.x());
        t_autoShuttle = new SendableTrigger(driver.b(), "autoShuttle");
        t_openBayDoor = new SendableTrigger(operator.povDown(), "openBayDoor");
        t_closeBayDoor = new SendableTrigger(operator.povUp(), "closeBayDoor");
        t_switchRelativeReference = new SendableTrigger(driver.leftBumper(), "switchRelativeReference");
        t_resetGyro = new SendableTrigger(driver.povDown(), "resetGyro");
        t_climb = new SendableTrigger(operator.rightTrigger(), "climb");
        t_climb_home = new SendableTrigger(operator.start().and(operator.back()), "climb_home");

        // Conditional Triggers
        t_hubLaunchValid = new SendableTrigger(
                () -> hubLaunchParameters.get().map(parameters -> parameters.isValid()).orElse(false),
                "nc_hubLaunchValid");

        t_snowBlowValid = new SendableTrigger(
                () -> snowBlowLaunchParameters.get().map(parameters -> parameters.isValid()).orElse(false),
                "nc_snowBlowValid");

        t_attemptToScore = new SendableTrigger(t_autoScore.or(t_partialManualScore).or(t_manualScore),
                "nc_attemptToScore");
        t_onAllianceSide = new SendableTrigger(drive.onAllianceSide.and(() -> DriverStation.isTeleop()),
                "nc_onAllianceSide");

        t_prepareFuel = new SendableTrigger(t_onAllianceSide.and(t_attemptToScore.negate()), "nc_prepareFuel");

        t_autoUnjam = new SendableTrigger(() -> false, "t_autoUnjam"); // TODO: Fix auto unjam.

        t_unjam = new SendableTrigger(t_autoUnjam.or(t_manualUnjam).negate(), "t_unjam");

        // Command Triggers
        cmd_autoScore_launchFuel = new SendableTrigger(
                t_autoScore
                        .and(t_onAllianceSide),
                "cmd_autoScore_launchFuel");
        cmd_autoScore_indexFuel = new SendableTrigger(
                t_autoScore
                        .and(t_hubLaunchValid)
                        .and(t_onAllianceSide)
                        .and(t_unjam.negate()),
                "cmd_autoScore_indexFuel");
        cmd_snowBlow_launchFuel = new SendableTrigger(
                t_autoSnowBlow
                        .and(t_onAllianceSide.negate()),
                "cmd_snowBlow_launchFuel");

        cmd_snowBlow_indexFuel = new SendableTrigger(
                t_autoSnowBlow
                        .and(t_onAllianceSide.negate())
                        .and(t_snowBlowValid)
                        .and(t_unjam.negate()),
                "cmd_snowBlow_launchFuel");

        cmd_autoIntake = new SendableTrigger(t_autoIntake.getTrigger().and(t_climb.negate()).and(t_autoShuttle),
                "cmd_autoIntake");
        cmd_autoShuttle = new SendableTrigger(t_autoShuttle.and(t_climb.negate()), "cmd_autoIntake");
        cmd_switchRelativeReference = new SendableTrigger(driver.leftBumper(), "cmd_switchRelativeReference");
        cmd_autoUnjam = new SendableTrigger(t_autoUnjam, "cmd_autoUnjam");
        cmd_manualUnjam = new SendableTrigger(t_manualUnjam, "cmd_manualUnjam");
        cmd_prepareFuel = new SendableTrigger(t_prepareFuel, "cmd_prepareFuel");
        cmd_bayDoor_open = new SendableTrigger(t_openBayDoor, "cmd_bayDoor_open");
        cmd_bayDoor_close = new SendableTrigger(t_closeBayDoor, "cmd_bayDoor_close");
    }
}
package frc.robot;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// Reference: https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src%2Fmain%2Fjava%2Forg%2Flittletonrobotics%2Ffrc2026%2Futil%2FHubShiftUtil.java
public class HubUtil {
    public enum GameState {
        UNKNOWN,
        AUTONOMOUS,
        TRANSITION,
        TELEOPERATED,
        ENDGAME
    }

    public enum TeleOperatedState {
        UNKNOWN,
        RED_HUB_INACTIVE,
        RED_HUB_ACTIVE,
        BLUE_HUB_INACTIVE,
        BLUE_HUB_ACTIVE,
    }

    public record HubInfo(GameState gameState, TeleOperatedState hubState, Time untilInactive) {
    }

    private static Timer timer = new Timer();

    // TODO: Properly implement HubUtil.
    // public static Alliance getHubActiveAlliance() {

    // }
}

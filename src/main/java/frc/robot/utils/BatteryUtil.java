package frc.robot.utils;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj.RobotController;

public class BatteryUtil {
    public static Power getPower(Current current) {
        return current.times(RobotController.getMeasureBatteryVoltage());
    }
}
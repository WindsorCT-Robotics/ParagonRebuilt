package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtil {
    public static Optional<Angle> angleOffset(Angle angle) {
        return DriverStation.getAlliance().map(alliance -> {
            if (alliance == Alliance.Red) {
                return AngleUtil.invert(angle);
            }

            return angle;
        });
    }
}
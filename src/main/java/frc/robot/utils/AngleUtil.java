package frc.robot.utils;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;

public class AngleUtil {
    public static Angle wrap(Angle angle) {
        return Radians.of(MathUtil.angleModulus(angle.in(Radians)));
    }
}

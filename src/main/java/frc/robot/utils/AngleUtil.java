package frc.robot.utils;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;

public class AngleUtil {
    /**
     * Wraps the angle within [-pi, pi].
     * 
     * @param angle
     * @return A wrapped angle between [-pi, pi].
     */
    public static Angle wrap(Angle angle) {
        return Radians.of(MathUtil.angleModulus(angle.in(Radians)));
    }

    public static Angle invert(Angle angle) {
        return wrap(angle.plus(Radians.of(Math.PI)));
    }
}

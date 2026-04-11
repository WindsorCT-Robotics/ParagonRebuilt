package frc.robot.utils;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Dimensionless;

public class ControllerUtil {
    public static Dimensionless curveAxis(Dimensionless percent, double exponent) {
        return Value.of(
                Math.abs(Math.pow(
                        percent.in(Value), exponent - 1)) * percent.unaryMinus().in(Value));
    }

    public static Dimensionless getAxisWithDeadBandAndCurve(Dimensionless value, Dimensionless deadband, double curve) {
        return curveAxis(Value.of(MathUtil.applyDeadband(
                value.in(Value),
                deadband.in(Value))),
                curve);
    }

    public static Dimensionless getAxisWithDeadBandAndCurve(double value, Dimensionless deadband, double curve) {
        return curveAxis(Value.of(MathUtil.applyDeadband(
                value,
                deadband.in(Value))),
                curve);
    }
}
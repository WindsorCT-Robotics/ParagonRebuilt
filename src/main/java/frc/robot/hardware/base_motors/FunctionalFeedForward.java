package frc.robot.hardware.base_motors;

import edu.wpi.first.units.measure.Voltage;

@FunctionalInterface
public interface FunctionalFeedForward {
    Voltage calculateWithVelocities(double currentAngle, double currentVelocity, double nextVelocity);
}

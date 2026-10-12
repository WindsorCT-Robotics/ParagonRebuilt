package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public boolean connected = false;
        public double positionRotations = 0.0;
        public double velocityRPM = 0.0;
        public double voltageVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    public default void updateInputs(SpindexerIOInputs i) {}
    public default void setVelocity(AngularVelocity velocity) {}
    public default void stop() {}
}

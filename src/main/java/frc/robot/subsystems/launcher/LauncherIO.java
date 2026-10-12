package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface LauncherIO {
    @AutoLog
    public static class LauncherIOInputs {
        public boolean leftMotorConnected = false;
        public boolean rightMotorConnected = false;

        public double leftPositionRotations = 0.0;
        public double leftVelocityRPM = 0.0;
        public double leftVoltageVolts = 0.0;
        public double leftStatorCurrentAmps = 0.0;
        public double leftSupplyCurrentAmps = 0.0;
        public double leftTemperatureCelsius = 0.0;

        public double rightPositionRotations = 0.0;
        public double rightVelocityRPM = 0.0;
        public double rightVoltageVolts = 0.0;
        public double rightStatorCurrentAmps = 0.0;
        public double rightSupplyCurrentAmps = 0.0;
        public double rightTemperatureCelsius = 0.0;
    }

    public default void updateInputs(LauncherIOInputs inputs) {}
    public default void setVelocity(AngularVelocity velocity) {}
    public default void stop() {}
}

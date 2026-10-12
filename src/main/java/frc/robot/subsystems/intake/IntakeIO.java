package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public double positionRotations = 0.0;
    public double velocityRPM = 0.0;
    public double voltageVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
  public default void setVelocity(AngularVelocity velocity) {}
  public default void stop() {}
}

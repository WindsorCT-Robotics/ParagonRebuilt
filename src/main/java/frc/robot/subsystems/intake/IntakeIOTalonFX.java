package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.CanID;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX motor;
  private final MotionMagicVelocityDutyCycle velocityRequest = new MotionMagicVelocityDutyCycle(0);

  public IntakeIOTalonFX(CanID can) {
    motor = new TalonFX(can.ID());
    motor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(null)
                        .withSupplyCurrentLimit(null)));
  }

  @Override
  public void updateInputs(IntakeIOInputs i) {
    i.connected = motor.isConnected();
    i.positionRotations = motor.getPosition().getValueAsDouble();
    i.velocityRPM = motor.getVelocity().getValueAsDouble();
    i.voltageVolts = motor.getMotorVoltage().getValueAsDouble();
    i.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    i.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    i.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}

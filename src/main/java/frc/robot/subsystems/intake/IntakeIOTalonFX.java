package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.CanID;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX motor;
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

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
    i.hasPiece = false; // Just a test
    i.positionRotations = motor.getPosition().getValueAsDouble();
    i.velocityRPM = motor.getVelocity().getValueAsDouble();
    i.voltageVolts = motor.getMotorVoltage().getValueAsDouble();
    i.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    i.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    i.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void resetEncoder() {
    motor.setPosition(0);
  }

  @Override
  public void setDutyCycle(Dimensionless percent) {
    motor.setControl(dutyCycleRequest.withOutput(percent.in(Value)));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage.in(Volts));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}

package frc.robot.hardware.base_motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.interfaces.IMotor;
import frc.robot.hardware.CanId;

public abstract class TalonFXMotorBase implements IMotor, Sendable {
    // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/configuration.html
    private final TalonFX motor;
    private final TalonFXConfigurator motorConfigurator;
    private final AngularVelocity maxAngularVelocity;

    protected TalonFXMotorBase(
        String name,
        CanId canId,
        NeutralModeValue neutralMode,
        InvertedValue inverted,
        Current currentLimit,
        AngularVelocity maxAngularVelocity
    ) {
        motor = new TalonFX(canId.Id());
        motorConfigurator = motor.getConfigurator();
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
            .withNeutralMode(neutralMode)
            .withInverted(inverted)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(currentLimit)
            .withStatorCurrentLimitEnable(true));
        motorConfigurator.apply(motorConfiguration);
        this.maxAngularVelocity = maxAngularVelocity;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setActuator(true);
        builder.setSafeState(this::stop);

        builder.addDoubleProperty("Voltage (V)", () -> getVoltage().in(Volts), this::setVoltage);
        builder.addDoubleProperty("Current (Amps)", () -> getCurrent().in(Amps), null);
        builder.addBooleanProperty("Is Motor Moving?", this::isMoving, null);
        builder.addDoubleProperty("Target Duty Cycle %", this::getDutyCycle, this::setDutyCycle);
        builder.addDoubleProperty("RPM (Rotations Per Minute)", () -> getAngularVelocity().in(RPM),
                this::setRPM);
        builder.addDoubleProperty("Temperature (C)", () -> getTemperature().in(Celsius), null);
        
    }

    @Override
    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    public void setVoltate(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public boolean isMoving() {
        return !motor.getAcceleration().getValue().equals(RotationsPerSecondPerSecond.of(0));
    }

    @Override
    public void resetRelativeEncoder() {
        motor.setPosition(Degrees.of(0));
    }

    @Override
    public void setDutyCycle(Dimensionless percentage) {
        motor.set(MathUtil.clamp(percentage.in(Percent), -1, 1));
    }

    private void setDutyCycle(double percentage) {
        motor.set(MathUtil.clamp(percentage, -1, 1));
    }

    private double getDutyCycle() {
        return motor.getDutyCycle().getValue();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    public Current getCurrent() {
        return motor.getTorqueCurrent().getValue();
    }

    public void setRPM(double rpm) {
        motor.set(rpm / maxAngularVelocity.in(RPM));
    }

    public AngularVelocity getAngularVelocity() {
        return motor.getVelocity().getValue();
    }

    private Temperature getTemperature() {
        return motor.getDeviceTemp().getValue();
    }
}
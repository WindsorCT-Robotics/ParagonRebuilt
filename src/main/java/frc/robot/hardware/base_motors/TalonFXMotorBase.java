package frc.robot.hardware.base_motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.hardware.CanId;
import frc.robot.interfaces.IClosedLoopMotor;

public class TalonFXMotorBase implements IClosedLoopMotor<TalonFX, TalonFXConfiguration>, Sendable {
    protected final TalonFX motor;
    private final TalonFXConfigurator configurator;
    private final TalonFXConfiguration configuration;
    private final Consumer<Dimensionless> dutyCycleSetter;
    private final Consumer<Voltage> voltageSetter;

    public TalonFXMotorBase(
            CanId canId,
            TalonFXConfiguration configuration,
            Consumer<Dimensionless> dutyCycleSetter,
            Consumer<Voltage> voltageSetter) {
        motor = new TalonFX(canId.Id());
        configurator = motor.getConfigurator();
        this.configuration = configuration;
        configurator.apply(configuration);
        this.dutyCycleSetter = dutyCycleSetter;
        this.voltageSetter = voltageSetter;
    }

    @Override
    public void setPointPosition(Angle angle) {
        motor.setControl(new MotionMagicVoltage(angle));
    }

    @Override
    public void setPointVelocity(AngularVelocity angularVelocity) {
        motor.setControl(new MotionMagicVelocityVoltage(angularVelocity));
    }

    @Override
    public void configure(Consumer<TalonFX> config) {
        config.accept(motor);
    }

    @Override
    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    @Override
    public TalonFXConfiguration getConfiguration() {
        return configuration;
    }

    @Override
    public Dimensionless getDutyCycle() {
        return Percent.of(motor.getDutyCycle().getValue());
    }

    /**
     * Duty Cycles are proportional to battery life.
     */
    @Override
    public void setDutyCycle(Dimensionless percentage) {
        motor.set(percentage.in(Percent));
    }

    @Override
    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }

    @Override
    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public boolean isMoving() {
        return motor.getVelocity().getValueAsDouble() != 0;
    }

    @Override
    public void resetRelativeEncoder() {
        motor.setPosition(Degrees.zero());
    }

    @Override
    public Current getCurrent() {
        return motor.getTorqueCurrent().getValue();
    }

    @Override
    public Temperature getTemperarure() {
        return motor.getDeviceTemp().getValue();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setActuator(true);
        builder.setSafeState(this::stop);

        builder.addDoubleProperty("Angle (Rotations)", () -> getAngle().in(Rotations), null);
        builder.addDoubleProperty("Duty Cycle (%)", () -> getDutyCycle().in(Percent),
                dutyCycle -> dutyCycleSetter.accept(Percent.of(dutyCycle)));
        builder.addDoubleProperty("Velocity (RPS)", () -> getVelocity().in(RotationsPerSecond), null);
        builder.addBooleanProperty("Is Moving", this::isMoving, null);
        builder.addDoubleProperty("Voltage (V)", () -> getVoltage().in(Volts),
                voltage -> voltageSetter.accept(Volts.of(voltage)));
        builder.addDoubleProperty("Current (Amps)", () -> getCurrent().in(Amps), null);
        builder.addDoubleProperty("Temperature (C)", () -> getTemperarure().in(Celsius), null);
    }
}

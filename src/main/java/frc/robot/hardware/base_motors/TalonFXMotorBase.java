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
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.hardware.CanId;
import frc.robot.interfaces.IMotor;

public class TalonFXMotorBase implements IMotor<TalonFX, TalonFXConfiguration>, Sendable {
    private final TalonFX motor;
    private final TalonFXConfigurator configurator;
    private final TalonFXConfiguration configuration;
    private final SimpleMotorFeedforward feedforward;
    private final AngularVelocity maxAngularVelocity;
    private final Voltage maxVoltage;
    private final Dimensionless maxPercentage;

    public TalonFXMotorBase(
            String name,
            CanId canId,
            TalonFXConfiguration configuration,
            SimpleMotorFeedforward feedforward,
            AngularVelocity maxAngularVelocity,
            Voltage maxVoltage,
            Dimensionless maxPercentage) {
        SendableRegistry.add(this, name);
        motor = new TalonFX(canId.Id());
        configurator = motor.getConfigurator();
        this.configuration = configuration;
        configurator.apply(configuration);
        this.feedforward = feedforward;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxVoltage = maxVoltage;
        this.maxPercentage = maxPercentage;
    }

    @Override
    public void configure(Consumer<TalonFX> config) {
        config.accept(motor);
    }

    @Override
    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    // TODO: Possibly add this to telemetry?
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
        Dimensionless clampedPercentage = Percent
                .of(MathUtil.clamp(percentage.in(Percent), maxPercentage.unaryMinus().in(Percent),
                        maxPercentage.in(Percent)));
        motor.set(clampedPercentage.in(Percent));
    }

    @Override
    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }

    @Override
    public void setRPS(AngularVelocity velocity) {
        AngularVelocity clampedVelocity = RotationsPerSecond.of(
                MathUtil.clamp(
                        velocity.in(RotationsPerSecond),
                        maxAngularVelocity.unaryMinus().in(RotationsPerSecond),
                        maxAngularVelocity.in(RotationsPerSecond)));

        setVoltage(
                Volts.of(
                        feedforward.calculate(
                                clampedVelocity.in(RotationsPerSecond))));
    }

    @Override
    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        Voltage clampedVoltage = Volts
                .of(MathUtil.clamp(voltage.in(Volts), maxVoltage.unaryMinus().in(Volts), maxVoltage.in(Volts)));
        motor.setVoltage(clampedVoltage.in(Volts));
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
        builder.addDoubleProperty("Duty Cycle (%)", () -> getDutyCycle().in(Percent), null);
        builder.addDoubleProperty("Velocity (RPS)", () -> getVelocity().in(RotationsPerSecond), null);
        builder.addBooleanProperty("Is Moving", this::isMoving, null);
        builder.addDoubleProperty("Voltage (V)", () -> getVoltage().in(Volts), null);
        builder.addDoubleProperty("Current (Amps)", () -> getCurrent().in(Amps), null);
        builder.addDoubleProperty("Temperature (C)", () -> getTemperarure().in(Celsius), null);
    }
}

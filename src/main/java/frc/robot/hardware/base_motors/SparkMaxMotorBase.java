package frc.robot.hardware.base_motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

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

public class SparkMaxMotorBase implements IMotor<SparkMax, SparkMaxConfigAccessor>, Sendable {
    private final SparkMax motor;
    private final SimpleMotorFeedforward feedforward;
    private final AngularVelocity maxAngularVelocity;
    private final Voltage maxVoltage;
    private final Dimensionless maxPercentage;

    protected SparkMaxMotorBase(
            String name,
            CanId canId,
            MotorType motorType,
            SimpleMotorFeedforward feedforward,
            SparkBaseConfig configuration,
            ResetMode resetMode,
            PersistMode persistMode,
            AngularVelocity maxAngularVelocity,
            Voltage maxVoltage,
            Dimensionless maxPercentage) {
        SendableRegistry.add(this, name);
        motor = new SparkMax(canId.Id(), motorType);
        motor.configure(configuration, resetMode, persistMode);
        this.feedforward = feedforward;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxVoltage = maxVoltage;
        this.maxPercentage = maxPercentage;
    }

    @Override
    public void configure(Consumer<SparkMax> config) {
        config.accept(motor);
    }

    @Override
    public Angle getAngle() {
        return Rotations.of(motor.getEncoder().getPosition());
    }

    // TODO: Possibly add this to telemetry?
    @Override
    public SparkMaxConfigAccessor getConfiguration() {
        return motor.configAccessor;
    }

    @Override
    public Dimensionless getDutyCycle() {
        return Value.of(motor.getAppliedOutput());
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
        return RPM.of(motor.getEncoder().getVelocity());
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
        return Volts.of(motor.getBusVoltage());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        Voltage clampedVoltage = Volts
                .of(MathUtil.clamp(voltage.in(Volts), maxVoltage.unaryMinus().in(Volts), maxVoltage.in(Volts)));
        motor.setVoltage(clampedVoltage.in(Volts));
    }

    @Override
    public boolean isMoving() {
        return motor.getEncoder().getVelocity() != 0;
    }

    @Override
    public void resetRelativeEncoder() {
        motor.getEncoder().setPosition(0);

    }

    @Override
    public Current getCurrent() {
        return Amps.of(motor.getOutputCurrent());
    }

    @Override
    public Temperature getTemperarure() {
        return Celsius.of(motor.getMotorTemperature());
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
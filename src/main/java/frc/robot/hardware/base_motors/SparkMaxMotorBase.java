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
    protected final SparkMax motor;
    private final FunctionalFeedForward ff;
    private final AngularVelocity maxAngularVelocity;
    private final Voltage maxVoltage;
    private final Dimensionless maxDutyCycle;

    private final Consumer<Dimensionless> dutyCycleSetter;
    private final Consumer<AngularVelocity> angularVelocitySetter;
    private final Consumer<Voltage> voltageSetter;

    protected SparkMaxMotorBase(
            String name,
            CanId canId,
            FunctionalFeedForward ff,
            SparkBaseConfig configuration,
            ResetMode resetMode,
            PersistMode persistMode,
            AngularVelocity maxAngularVelocity,
            Voltage maxVoltage,
            Dimensionless maxDutyCycle,
            Consumer<Dimensionless> dutyCycleSetter,
            Consumer<AngularVelocity> angularVelocitySetter,
            Consumer<Voltage> voltageSetter) {
        SendableRegistry.addLW(this, name);
        motor = new SparkMax(canId.Id(), MotorType.kBrushless);
        motor.configure(configuration, resetMode, persistMode);
        this.ff = ff;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxVoltage = maxVoltage;
        this.maxDutyCycle = maxDutyCycle;
        this.dutyCycleSetter = dutyCycleSetter;
        this.angularVelocitySetter = angularVelocitySetter;
        this.voltageSetter = voltageSetter;
    }

    @Override
    public void configure(Consumer<SparkMax> config) {
        config.accept(motor);
    }

    @Override
    public Angle getAngle() {
        return Rotations.of(motor.getEncoder().getPosition());
    }

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
        // Dimensionless clampedPercentage = Percent
        // .of(MathUtil.clamp(percentage.in(Percent),
        // maxDutyCycle.unaryMinus().in(Percent),
        // maxDutyCycle.in(Percent)));

        motor.set(percentage.in(Value));
    }

    @Override
    public AngularVelocity getVelocity() {
        return RPM.of(motor.getEncoder().getVelocity());
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        AngularVelocity clampedVelocity = RotationsPerSecond.of(
                MathUtil.clamp(
                        velocity.in(RotationsPerSecond),
                        maxAngularVelocity.unaryMinus().in(RotationsPerSecond),
                        maxAngularVelocity.in(RotationsPerSecond)));

        setVoltage(
                ff.calculateWithVelocities(
                        getAngle().in(Rotations),
                        getVelocity().in(RotationsPerSecond),
                        clampedVelocity.in(RotationsPerSecond)));
    }

    @Override
    public Voltage getVoltage() {
        return Volts.of(motor.getBusVoltage());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        Voltage clampedVoltage = Volts.of(
                MathUtil.clamp(
                        voltage.in(Volts),
                        maxVoltage.unaryMinus().in(Volts),
                        maxVoltage.in(Volts)));
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

        builder.addDoubleProperty("Encoder Value", () -> getAngle().in(Rotations), null);
        builder.addDoubleProperty("Duty Cycle (%)", () -> getDutyCycle().in(Percent),
                val -> dutyCycleSetter.accept(Percent.of(val)));
        builder.addDoubleProperty("Velocity (RPS)", () -> getVelocity().in(RotationsPerSecond),
                val -> angularVelocitySetter.accept(RotationsPerSecond.of(val)));
        builder.addBooleanProperty("Is Moving", this::isMoving, null);
        builder.addDoubleProperty("Voltage (V)", () -> getVoltage().in(Volts),
                val -> voltageSetter.accept(Volts.of(val)));
        builder.addDoubleProperty("Current (Amps)", () -> getCurrent().in(Amps), null);
        builder.addDoubleProperty("Temperature (C)", () -> getTemperarure().in(Celsius), null);
    }
}
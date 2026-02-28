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
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

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

public class SparkMaxMotorBase implements IClosedLoopMotor<SparkMax, SparkMaxConfigAccessor>, Sendable {
    protected final SparkMax motor;

    private final String name;
    private final Consumer<Dimensionless> dutyCycleSetter;
    private final Consumer<Voltage> voltageSetter;

    protected SparkMaxMotorBase(
            String name,
            CanId canId,
            SparkBaseConfig configuration,
            ResetMode resetMode,
            PersistMode persistMode,
            Consumer<Dimensionless> dutyCycleSetter,
            Consumer<Voltage> voltageSetter) {
        motor = new SparkMax(canId.Id(), MotorType.kBrushless);
        motor.configure(configuration, resetMode, persistMode);
        this.dutyCycleSetter = dutyCycleSetter;
        this.voltageSetter = voltageSetter;
        this.name = name;
    }

    public void follow(int id, boolean invert) {
        SparkBaseConfig config = new SparkMaxConfig().follow(id, invert);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pauseFollower() {
        motor.pauseFollowerMode();
    }

    public void resumeFollower() {
        motor.resumeFollowerMode();
    }

    public int getId() {
        return motor.getDeviceId();
    }

    @Override
    public String getSmartDashboardName() {
        return name;
    }

    @Override
    public void setPointPosition(Angle angle) {
        motor.getClosedLoopController().setSetpoint(angle.in(Rotations), ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void setPointVelocity(AngularVelocity angularVelocity) {
        motor.getClosedLoopController().setSetpoint(angularVelocity.in(RPM),
                ControlType.kMAXMotionVelocityControl);
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
        motor.set(percentage.in(Percent));
    }

    @Override
    public AngularVelocity getVelocity() {
        return RPM.of(motor.getEncoder().getVelocity());
    }

    @Override
    public Voltage getVoltage() {
        return Volts.of(motor.getBusVoltage());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
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

    // TODO: Add a way to set a setpoint. Possibly add as a interface?

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setActuator(true);
        builder.setSafeState(this::stop);

        builder.addDoubleProperty("Encoder Value", () -> getAngle().in(Rotations), null);
        builder.addDoubleProperty("Duty Cycle (%)", () -> getDutyCycle().in(Percent),
                val -> dutyCycleSetter.accept(Percent.of(val)));
        builder.addDoubleProperty("Velocity (RPS)", () -> getVelocity().in(RotationsPerSecond), null);
        builder.addBooleanProperty("Is Moving", this::isMoving, null);
        builder.addDoubleProperty("Voltage (V)", () -> getVoltage().in(Volts),
                val -> voltageSetter.accept(Volts.of(val)));
        builder.addDoubleProperty("Current (Amps)", () -> getCurrent().in(Amps), null);
        builder.addDoubleProperty("Temperature (C)", () -> getTemperarure().in(Celsius), null);
    }
}
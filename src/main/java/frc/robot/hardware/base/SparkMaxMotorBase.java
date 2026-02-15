package frc.robot.hardware.base;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
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

public abstract class SparkMaxMotorBase implements IMotor, Sendable {
    protected final SparkMax motor;
    protected final SparkBaseConfig motorConfiguration;
    protected final SparkClosedLoopController closedLoopController;
    protected final CanId canId;
    private final AngularVelocity maxAngularVelocity;

    private static final Dimensionless MAX_DUTY = Percent.of(100);
    private static final Dimensionless MIN_DUTY = Percent.of(-100);

    /**
     * 
     * @param name
     * @param canId
     * @param motorType
     * @param maxAngularVelocity
     * @param motorConfiguration
     * @param resetMode          It's recommended that when initalized, it should be
     *                           set to `kResetSafeParameters`
     * @param persistMode        It's recommended that when initalized, it should be
     *                           set to `kPersistParameters`
     */
    protected SparkMaxMotorBase(
            String name,
            CanId canId,
            MotorType motorType,
            AngularVelocity maxAngularVelocity,
            SparkBaseConfig motorConfiguration,
            ResetMode resetMode,
            PersistMode persistMode) {
        SendableRegistry.add(this, name);
        this.canId = canId;
        this.maxAngularVelocity = maxAngularVelocity;
        this.motorConfiguration = motorConfiguration;
        motor = new SparkMax(canId.Id(), motorType);
        motor.configure(this.motorConfiguration, resetMode, persistMode);
        closedLoopController = motor.getClosedLoopController();
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
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void resetRelativeEncoder() {
        motor.getEncoder().setPosition(0);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public Voltage getVoltage() {
        return Volts.of(motor.getBusVoltage());
    }

    @Override
    public boolean isMoving() {
        return (motor.getEncoder().getVelocity() != 0);
    }

    @Override
    public void setDutyCycle(Dimensionless percentage) {
        if (percentage.gt(MAX_DUTY) || percentage.lt(MIN_DUTY)) {
            throw new IllegalArgumentException("Percentage " + percentage
                    + " is out of bounds for duty motor Acceptable ranges are [" + MIN_DUTY + ", " + MAX_DUTY + "].");
        }

        motor.set(percentage.in(Percent));
    }

    private void setDutyCycle(double percentage) {
        double clampPercentage = MathUtil.clamp(percentage, MIN_DUTY.in(Percent), MAX_DUTY.in(Percent));
        setDutyCycle(Percent.of(clampPercentage));
    }

    private double getDutyCycle() {
        return motor.getAppliedOutput();
    }

    private void setRPM(double rpm) {
        setDutyCycle(Percent.of(rpm / maxAngularVelocity.in(RPM)));
    }

    private AngularVelocity getAngularVelocity() {
        return RPM.of(motor.getEncoder().getVelocity());
    }

    private Temperature getTemperature() {
        return Celsius.of(motor.getMotorTemperature());
    }

    private Current getCurrent() {
        return Amps.of(motor.getOutputCurrent());
    }
}
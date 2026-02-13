package frc.robot.hardware;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;

import frc.robot.interfaces.IMotor;

public class KickerMotor implements IMotor, Sendable {
    private final SparkMaxConfig configuaration;
    private final SparkMax motor;
    private static final Dimensionless MAX_DUTY = Percent.of(100);
    private static final Dimensionless MIN_DUTY = Percent.of(-100);

    // Data:
    // https://docs.wcproducts.com/welcome/electronics/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance#trapezoidal-commutation
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RPM.of(6000);

    public KickerMotor(
            String name,
            CanId canId) {
        SendableRegistry.add(this, name);

        configuaration = new SparkMaxConfig();
        configuaration.idleMode(IdleMode.kBrake);
        motor = new SparkMax(canId.Id(), MotorType.kBrushless);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setActuator(true);
        builder.setSafeState(this::stop);

        builder.addDoubleProperty("Voltage (V)", () -> getVoltage().in(Volts), this::setVoltage);
        builder.addDoubleProperty("Current (Amps)", () -> getCurrent().in(Amps), null);
        builder.addBooleanProperty("Is Motor Moving?", () -> isMoving(), null);
        builder.addDoubleProperty("Target Duty Cycle %", () -> motor.getAppliedOutput(), this::setDutyCycle);
        builder.addDoubleProperty("RPM (Rotations Per Minute)", () -> getAngularVelocity().in(RPM),
                this::setRPM);
        builder.addDoubleProperty("Temperature (C)", () -> getTemperature().in(Celsius), null);
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
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void resetRelativeEncoder() {
        motor.getEncoder().setPosition(0);
    }

    @Override
    public void setDutyCycle(Dimensionless percentage) {
        if (percentage.gt(MAX_DUTY) || percentage.lt(MIN_DUTY)) {
            throw new IllegalArgumentException("Percentage " + percentage
                    + " is out of bounds for duty motor Acceptable ranges are [" + MIN_DUTY + ", " + MAX_DUTY + "].");
        }

        motor.set(percentage.in(Percent));
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

    private void setDutyCycle(double percentage) {
        if (percentage > 1) {
            setDutyCycle(MAX_DUTY);
        } else if (percentage < -1) {
            setDutyCycle(MIN_DUTY);
        } else {
            setDutyCycle(Percent.of(percentage));
        }
    }

    private void setRPM(double rpm) {
        setDutyCycle(Percent.of(rpm / MAX_ANGULAR_VELOCITY.in(RPM)));
    }
}
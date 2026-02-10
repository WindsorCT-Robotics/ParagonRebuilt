package frc.robot.hardware;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Percent;
import frc.robot.interfaces.IMotor;

public class KickerMotor implements IMotor, Sendable {
    private final SparkMaxConfig configuaration;
    private final SparkMax motor;
    private static final Dimensionless MAX_DUTY = Percent.of(100);
    private static final Dimensionless MIN_DUTY = Percent.of(-100);

    public KickerMotor(CanId canId) {
        configuaration = new SparkMaxConfig();
        configuaration.idleMode(IdleMode.kBrake);
        motor = new SparkMax(canId.Id(), MotorType.kBrushless);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setActuator(true);
        builder.setSafeState(this::stop);
        builder.addDoubleProperty("Voltage (V)", () -> getVoltage(), null);
        builder.addBooleanProperty("Is Motor Moving?", () -> isMoving(), null);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public double getVoltage() {
        return motor.getBusVoltage();
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
        motor.set(percentage.in(Value));
    }
}
package frc.robot.hardware;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Percent;
import frc.robot.interfaces.IDutyCycleMotor;

public class KickerMotor implements IDutyCycleMotor {
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
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void getVoltage() {
        motor.getBusVoltage();
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
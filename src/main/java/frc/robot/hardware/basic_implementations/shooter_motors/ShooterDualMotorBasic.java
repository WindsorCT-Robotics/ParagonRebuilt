package frc.robot.hardware.basic_implementations.shooter_motors;

import frc.robot.interfaces.IMotor;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.CanId;

public class ShooterDualMotorBasic implements IMotor {
    private final ShooterMotorBasic leftMotor;
    private final ShooterMotorBasic rightMotor;

    public ShooterDualMotorBasic(
        CanId leftMotorCanId,
        CanId rightMotorCanId
    ) {
        leftMotor = new ShooterMotorBasic("Left Shooter Motor", leftMotorCanId);
        rightMotor = new ShooterMotorBasic("Right Shooter Motor", rightMotorCanId);
    }

    @Override
    public Voltage getVoltage() {
        return leftMotor.getVoltage().plus(rightMotor.getVoltage()).div(2);
    }

    @Override
    public boolean isMoving() {
        return leftMotor.isMoving() || rightMotor.isMoving();
    }

    @Override
    public void resetRelativeEncoder() {
        leftMotor.resetRelativeEncoder();
        rightMotor.resetRelativeEncoder();
    }

    @Override
    public void setDutyCycle(Dimensionless percentage) {
        leftMotor.setDutyCycle(percentage);
        rightMotor.setDutyCycle(percentage);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }

    public void setInverted(InvertedValue inverted) {
        leftMotor.setInverted(inverted);
        rightMotor.setInverted((inverted == InvertedValue.Clockwise_Positive) ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);
    }
}
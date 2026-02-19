package frc.robot.hardware.basic_implementations.intake_motors;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.CanId;
import frc.robot.interfaces.IMotor;

public class BayDoorDualMotorBasic implements IMotor {
    private final BayDoorMotorBasic leftMotor;
    private final BayDoorMotorBasic rightMotor;
    private final static boolean INVERTED = true;

    public BayDoorDualMotorBasic(
            String name,
            CanId leftMotorId,
            CanId rightMotorId) {
        leftMotor = new BayDoorMotorBasic(name, leftMotorId);
        rightMotor = new BayDoorMotorBasic(name, rightMotorId);
        leftMotor.setInverted(INVERTED);
        rightMotor.setInverted(!INVERTED);
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

    public void setLeftMotorDutyCycle(Dimensionless percentage) {
        leftMotor.setDutyCycle(percentage);
    }

    public void setRightMotorDutyCycle(Dimensionless percentage) {
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

    public void stopLeftMotor() {
        leftMotor.stop();
    }

    public void stopRightMotor() {
        rightMotor.stop();
    }

    public boolean atLeftMotorSoftForwardLimit() {
        return leftMotor.atSoftForwardLimit();
    }

    public boolean atRightMotorSoftForwardLimit() {
        return rightMotor.atSoftForwardLimit();
    }

    public boolean atLeftMotorSoftReverseLimit() {
        return leftMotor.atSoftReverseLimit();
    }

    public boolean atRightMotorSoftReverseLimit() {
        return rightMotor.atSoftReverseLimit();
    }

    public void setIdleMode(IdleMode idleMode) {
        leftMotor.setIdleMode(idleMode);
        rightMotor.setIdleMode(idleMode);
    }

    public Angle getLeftRotation() {
        return leftMotor.getRotation();
    }

    public Angle getRightRotation() {
        return rightMotor.getRotation();
    }

    public Angle getRotation() {
        return leftMotor.getRotation().plus(rightMotor.getRotation()).div(2);
    }
}

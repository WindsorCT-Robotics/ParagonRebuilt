package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.CanID;

public class LauncherIOTalonFX implements LauncherIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFXConfiguration generalConfig = new TalonFXConfiguration();
    private final MotionMagicVelocityDutyCycle velocityDutyCycle = new MotionMagicVelocityDutyCycle(0);

    public LauncherIOTalonFX(CanID leftCan, CanID rightCan) {
        leftMotor = new TalonFX(leftCan.ID());
        rightMotor = new TalonFX(rightCan.ID());

        leftMotor.getConfigurator().apply(generalConfig);
        rightMotor.getConfigurator().apply(generalConfig);

        rightMotor.setControl(new Follower(leftCan.ID(), null));
    }

    @Override
    public void updateInputs(LauncherIOInputs i) {
        i.leftMotorConnected = leftMotor.isConnected();
        i.leftPositionRotations = leftMotor.getPosition().getValueAsDouble();
        i.leftVelocityRPM = leftMotor.getVelocity().getValueAsDouble();
        i.leftVoltageVolts = leftMotor.getMotorVoltage().getValueAsDouble();
        i.leftStatorCurrentAmps = leftMotor.getStatorCurrent().getValueAsDouble();
        i.leftSupplyCurrentAmps = leftMotor.getSupplyCurrent().getValueAsDouble();
        i.leftTemperatureCelsius = leftMotor.getDeviceTemp().getValueAsDouble();

        i.rightMotorConnected = rightMotor.isConnected();
        i.rightPositionRotations = rightMotor.getPosition().getValueAsDouble();
        i.rightVelocityRPM = rightMotor.getVelocity().getValueAsDouble();
        i.rightVoltageVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        i.rightStatorCurrentAmps = rightMotor.getStatorCurrent().getValueAsDouble();
        i.rightSupplyCurrentAmps = rightMotor.getSupplyCurrent().getValueAsDouble();
        i.rightTemperatureCelsius = rightMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        leftMotor.setControl(velocityDutyCycle.withVelocity(velocity));
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
    }
}

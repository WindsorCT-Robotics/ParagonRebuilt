package frc.robot.subsystems.baydoor;

import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.CanID;

public class BaydoorIOTalonFX implements BaydoorIO {
    public TalonFX leftMotor;
    public TalonFX rightMotor;

    public PositionDutyCycle leftTargetPosition = new PositionDutyCycle(0);
    public PositionDutyCycle rightTargetPosition = new PositionDutyCycle(0);

    public DutyCycleOut leftTargetDutyCycle = new DutyCycleOut(0);
    public DutyCycleOut rightTargetDutyCycle = new DutyCycleOut(0);

    public BaydoorIOTalonFX(CanID leftCanID, CanID rightCanID) {
        leftMotor = new TalonFX(leftCanID.ID());
        rightMotor = new TalonFX(rightCanID.ID());
    }

    @Override
    public void setPosition(Angle angle) {
        leftMotor.setControl(leftTargetPosition.withPosition(angle));
        rightMotor.setControl(rightTargetPosition.withPosition(angle));
    }

    @Override
    public void setDutyCycle(Dimensionless percent) {
        leftMotor.setControl(leftTargetDutyCycle.withOutput(percent.in(Value)));
        rightMotor.setControl(rightTargetDutyCycle.withOutput(percent.in(Value)));
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void updateInputs(BaydoorIOInputs i) {
        i.leftMotorConnected = leftMotor.isConnected();
        i.rightMotorConnected = rightMotor.isConnected();

        i.leftPositionRotations = leftMotor.getPosition().getValueAsDouble();
        i.leftVelocityRPM = leftMotor.getVelocity().getValueAsDouble();
        i.leftVoltageVolts = leftMotor.getMotorVoltage().getValueAsDouble();
        i.leftStatorCurrentAmps = leftMotor.getStatorCurrent().getValueAsDouble();
        i.leftSupplyCurrentAmps = leftMotor.getSupplyCurrent().getValueAsDouble();
        i.leftTemperatureCelsius = leftMotor.getDeviceTemp().getValueAsDouble();

        i.rightPositionRotations = rightMotor.getPosition().getValueAsDouble();
        i.rightVelocityRPM = rightMotor.getVelocity().getValueAsDouble();
        i.rightVoltageVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        i.rightStatorCurrentAmps = rightMotor.getStatorCurrent().getValueAsDouble();
        i.rightSupplyCurrentAmps = rightMotor.getSupplyCurrent().getValueAsDouble();
        i.rightTemperatureCelsius = rightMotor.getDeviceTemp().getValueAsDouble();
    }    
}

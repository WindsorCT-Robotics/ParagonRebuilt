package frc.robot.hardware;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.interfaces.IDutyCycleMotor;

public class KickerMotor implements IDutyCycleMotor {
    private final CanId canId;
    private final SparkMaxConfig configuaration;
    private final SparkMax motor;

    public KickerMotor(CanId canId) {
        this.canId = canId;
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVoltage'");
    }

    @Override
    public void isMoving() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isMoving'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public void resetRelativeEncoder() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetRelativeEncoder'");
    }

    @Override
    public void setDutyCycle(Dimensionless percentage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDutyCycle'");
    }
}
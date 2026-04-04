package frc.robot.hardware.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorX44Base;
import frc.robot.interfaces.IHomingMotor;

public class ClimberMotor extends KrakenMotorX44Base implements IHomingMotor {
    public ClimberMotor(
            String name,
            CanId canId,
            TalonFXConfiguration configuration) {
        super(name, canId, configuration);
    }

    @Override
    public void home(boolean stopHoming, Dimensionless dutyCycle) {
        if (!stopHoming) {
            setDutyCycle(dutyCycle);
        } else {
            stop();
            resetRelativeEncoder();
        }
    }
}
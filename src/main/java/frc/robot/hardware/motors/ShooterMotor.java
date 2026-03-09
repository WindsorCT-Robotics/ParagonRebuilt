package frc.robot.hardware.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class ShooterMotor extends KrakenMotorBase {
        public ShooterMotor(
                        String name,
                        CanId canId,
                        TalonFXConfiguration configuration) {
                super(
                                name,
                                canId,
                                configuration);
        }
}
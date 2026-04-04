package frc.robot.hardware.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorX60Base;

public class SpindexterMotor extends KrakenMotorX60Base {
        public SpindexterMotor(
                        String name,
                        CanId canId,
                        TalonFXConfiguration configuration) {
                super(
                                name,
                                canId,
                                configuration);
        }
}
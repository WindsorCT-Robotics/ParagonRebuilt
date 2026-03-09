package frc.robot.hardware.spindexer_motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class SpindexterMotor extends KrakenMotorBase {
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
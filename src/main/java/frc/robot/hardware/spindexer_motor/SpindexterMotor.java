package frc.robot.hardware.spindexer_motor;

import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class SpindexterMotor extends KrakenMotorBase {
        public SpindexterMotor(
                        String name,
                        CanId canId) {
                super(
                                name,
                                canId);
        }
}
package frc.robot.hardware.basic_implementations.shooter_motors;

import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class ShooterMotorBasic extends KrakenMotorBase {
        public ShooterMotorBasic(
                        String name,
                        CanId canId) {
                super(
                                name,
                                canId);
        }
}
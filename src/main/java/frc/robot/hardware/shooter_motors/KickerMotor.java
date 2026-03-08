package frc.robot.hardware.shooter_motors;

import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class KickerMotor extends KrakenMotorBase {
        public KickerMotor(
                        String name,
                        CanId canId) {
                super(
                                name,
                                canId);
        }
}
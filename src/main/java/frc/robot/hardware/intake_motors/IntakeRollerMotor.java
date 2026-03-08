package frc.robot.hardware.intake_motors;

import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class IntakeRollerMotor extends KrakenMotorBase {
        public IntakeRollerMotor(
                        String name,
                        CanId canId) {
                super(
                                name,
                                canId);
        }
}
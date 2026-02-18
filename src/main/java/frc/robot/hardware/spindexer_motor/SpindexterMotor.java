package frc.robot.hardware.spindexer_motor;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class SpindexterMotor extends KrakenMotorBase {
    private static final Current CURRENT_LIMIT = DEFAULT_CURRENT;

    public SpindexterMotor(String name, CanId canId) {
        super(
                name,
                canId,
                NeutralModeValue.Brake,
                InvertedValue.Clockwise_Positive,
                CURRENT_LIMIT);
    }
}
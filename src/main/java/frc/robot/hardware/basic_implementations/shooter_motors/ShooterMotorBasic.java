package frc.robot.hardware.basic_implementations.shooter_motors;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class ShooterMotorBasic extends KrakenMotorBase {
    private static final Current CURRENT_LIMIT = DEFAULT_CURRENT;

    public ShooterMotorBasic(String name, CanId canId) {
        super(
                name,
                canId,
                NeutralModeValue.Brake,
                InvertedValue.Clockwise_Positive,
                CURRENT_LIMIT);
    }

    public void setInverted(InvertedValue inverted) {
        motorConfigurator.apply(new MotorOutputConfigs().withInverted(inverted));
    }
}
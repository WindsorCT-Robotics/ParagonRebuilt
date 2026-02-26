package frc.robot.hardware.spindexer_motor;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class SpindexterMotor extends KrakenMotorBase {
    public SpindexterMotor(
            String name,
            CanId canId,
            Consumer<Dimensionless> dutyCycleSetter,
            Consumer<Voltage> voltageSetter) {
        super(
                name,
                canId,
                new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive))
                        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(DEFAULT_CURRENT)
                                .withStatorCurrentLimitEnable(true)),
                dutyCycleSetter,
                voltageSetter);
    }
}
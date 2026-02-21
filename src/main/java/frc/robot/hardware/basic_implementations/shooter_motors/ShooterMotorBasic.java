package frc.robot.hardware.basic_implementations.shooter_motors;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class ShooterMotorBasic extends KrakenMotorBase {
    private static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 0); // TODO: Figure
                                                                                                  // velocity (kV)

    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(1); // TODO: Figure good speed.
    // These are zero because the Bay Door should only be controlled by setting the
    // rotations per second.
    private static final Voltage MAX_VOLTAGE = Volts.of(0);
    private static final Dimensionless MAX_PERCENTAGE = Percent.of(0);

    public ShooterMotorBasic(String name, CanId canId) {
        super(
                name,
                canId,
                new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive))
                        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(DEFAULT_CURRENT)
                                .withStatorCurrentLimitEnable(true)),
                feedforward,
                MAX_ANGULAR_VELOCITY,
                MAX_VOLTAGE,
                MAX_PERCENTAGE);
    }
}
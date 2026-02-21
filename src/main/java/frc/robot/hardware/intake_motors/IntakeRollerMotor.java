package frc.robot.hardware.intake_motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.NeoMotorBase;

public class IntakeRollerMotor extends NeoMotorBase {
    private static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 0); // TODO: Figure
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(1); // TODO: Figure good speed.
    // These are zero because the Bay Door should only be controlled by setting the
    // rotations per second.
    private static final Voltage MAX_VOLTAGE = Volts.of(0);
    private static final Dimensionless MAX_PERCENTAGE = Percent.of(0);

    public IntakeRollerMotor(String name, CanId canId) {
        super(name, canId, feedforward,
                new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(
                        (int) DEFAULT_CURRENT.in(Amps)),
                ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters,
                MAX_ANGULAR_VELOCITY, MAX_VOLTAGE, MAX_PERCENTAGE);
    }
}
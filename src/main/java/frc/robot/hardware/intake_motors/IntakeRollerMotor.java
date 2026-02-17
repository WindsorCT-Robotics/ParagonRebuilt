package frc.robot.hardware.intake_motors;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Current;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeRollerMotor extends KrakenMotorBase {
    private static final IdleMode IDLE_MODE = IdleMode.kBrake;
    private static final ResetMode RESET_MODE = ResetMode.kResetSafeParameters;
    private static final PersistMode PERSIST_MODE = PersistMode.kPersistParameters;
    private static final boolean INVERTED = false;
    private static final Current CURRENT_LIMIT = Amps.of(0); // TODO: Get the recommended current limit of this motor.

    public IntakeRollerMotor(String name, CanId canId) {
        super(
                name,
                canId,
                new SparkMaxConfig()
                        .idleMode(IDLE_MODE)
                        .inverted(INVERTED)
                        .smartCurrentLimit((int) CURRENT_LIMIT.in(Amps)),
                RESET_MODE,
                PERSIST_MODE);

    }
}

package frc.robot.hardware;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Current;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeRollerMotor extends KrakenMotorBase {
    private static final IdleMode IDLE_MODE = IdleMode.kBrake;
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
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);

    }
}

package frc.robot.hardware.basic_implementations.intake_motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.NeoMotorBase;
import frc.robot.interfaces.IAngularPositionMotor;

public class BayDoorMotorBasic extends NeoMotorBase implements IAngularPositionMotor {
    private static final IdleMode IDLE_MODE = IdleMode.kBrake;
    private static final boolean INVERTED = false;
    private static final Current CURRENT_LIMIT = DEFAULT_CURRENT;
    private static final ResetMode RESET_MODE = ResetMode.kResetSafeParameters;
    private static final PersistMode PERSIST_MODE = PersistMode.kPersistParameters;
    private static final AngularVelocity POSITION_ANGULAR_VELOCITY = RPM.of(1);

    public static final Angle OPEN_ANGLE = Rotations.of(21.5);
    public static final Angle CLOSE_ANGLE = Rotations.of(0);

    public BayDoorMotorBasic(String name, CanId canId) {
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

    @Override
    public void setAngularPosition(Angle angle) {
        if (angle.gt(getRotation())) {
            setRPS(POSITION_ANGULAR_VELOCITY);
        }

        if (angle.lt(getRotation())) {
            setRPS(POSITION_ANGULAR_VELOCITY);
        }
    }

    public Dimensionless getDutyCycle() {
        return Percent.of(motor.get());
    }

    public void setIdleMode(IdleMode idleMode) {
        motorConfiguration.idleMode(idleMode);
    }

    private Angle getMotorPosition() {
        return Rotations.of(motor.getEncoder().getPosition());
    }

    public boolean atSoftForwardLimit() {
        return getMotorPosition().gte(OPEN_ANGLE);
    }

    public boolean atSoftReverseLimit() {
        return getMotorPosition().lte(CLOSE_ANGLE);
    }

    public AngularVelocity getVelocity() {
        return RPM.of(motor.getEncoder().getVelocity());
    }
}
package frc.robot.hardware.basic_implementations.intake_motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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

    public void setFollower(BayDoorMotorBasic leadMotor) {
        motorConfiguration.follow(leadMotor.canId.Id()); // TODO: Check if this actually follows the leadMotor.
    }

    @Override
    public void setAngularPosition(Angle angle) {
        if (angle.gt(getRotation())) {
            setRPM(POSITION_ANGULAR_VELOCITY.in(RPM));
        }

        if (angle.lt(getRotation())) {
            setRPM(POSITION_ANGULAR_VELOCITY.times(-1).in(RPM));
        }
    }
}
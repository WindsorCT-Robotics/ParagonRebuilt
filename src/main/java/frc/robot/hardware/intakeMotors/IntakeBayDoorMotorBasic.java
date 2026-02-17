package frc.robot.hardware.intakeMotors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hardware.CanId;
import frc.robot.hardware.baseMotors.NeoMotorBase;

public class IntakeBayDoorMotorBasic extends NeoMotorBase {
    
    public final Trigger isAtForwardLimit = new Trigger(this::isAtForwardSoftLimit);
    // public final Trigger isAtReverseLimit = new Trigger(this::isAtReverseLimit);
    private static final IdleMode IDLE_MODE = IdleMode.kBrake;
    private static final boolean INVERTED = false;
    private static final Current CURRENT_LIMIT = Amps.of(80);
    private static final ResetMode RESET_MODE = ResetMode.kResetSafeParameters;
    private static final PersistMode PERSIST_MODE = PersistMode.kPersistParameters;
    public static final Angle CLOSED_ANGLE = Degrees.of(0);
    public static final Angle OPENED_ANGLE = Degrees.of(0); // TODO: How much should the motor turn to become open to
                                                            // intake fuel?
    public static final Dimensionless ARM_SPEED = Percent.of(5);
    private static final Behavior REVERSE_LIMIT_BEHAVIOR = Behavior.kStopMovingMotorAndSetPosition;
    private static final Behavior FORWARD_LIMIT_BEHAVIOR = Behavior.kStopMovingMotorAndSetPosition;

    public IntakeBayDoorMotorBasic(String name, CanId canId) {
        super(
                name,
                canId,
                new SparkMaxConfig()
                        .idleMode(IDLE_MODE)
                        .inverted(INVERTED)
                        .smartCurrentLimit((int) CURRENT_LIMIT.in(Amps)),
                RESET_MODE,
                PERSIST_MODE);

        
        motorConfiguration.limitSwitch.reverseLimitSwitchTriggerBehavior(REVERSE_LIMIT_BEHAVIOR);
        motorConfiguration.limitSwitch.forwardLimitSwitchTriggerBehavior(FORWARD_LIMIT_BEHAVIOR);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle (Degrees)", () -> getAngle().in(Degrees), null);
    }

    public Angle getAngle() {
        return Rotations.of(motor.getEncoder().getPosition());
    }

    public void setFollower(IntakeBayDoorMotorBasic leadMotor) {
        motorConfiguration.follow(leadMotor.canId.Id());
    }

    public void setInverted(boolean inverted) {
        motorConfiguration.inverted(inverted);
    }

    // private boolean isAtReverseLimit() {
    //     return !homeLimit.get();
    // }

    private boolean isAtForwardSoftLimit() {
        return getAngle().gte(OPENED_ANGLE);
    }

    public void setIdleMode(IdleMode idleMode) {
        motorConfiguration.idleMode(idleMode);
    }
}

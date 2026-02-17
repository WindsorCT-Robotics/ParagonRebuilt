package frc.robot.hardware.intake_motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.NeoMotorBase;
import frc.robot.interfaces.IAngularPositionMotor;

public class IntakeBayDoorMotor extends NeoMotorBase implements IAngularPositionMotor {
    private static final IdleMode IDLE_MODE = IdleMode.kBrake;
    private static final boolean INVERTED = false;
    private static final ResetMode RESET_MODE = ResetMode.kResetSafeParameters;
    private static final PersistMode PERSIST_MODE = PersistMode.kPersistParameters;
    private static final Current CURRENT_LIMIT = Amps.of(0); // TODO: Get the recommended current limit of this motor.

    public static final Angle CLOSED_ANGLE = Degrees.of(0);
    public static final Angle OPENED_ANGLE = Degrees.of(0); // TODO: How much should the motor turn to become open to
                                                            // intake fuel?

    // https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control#feed-forward-constant-terms
    private static final ControlType setPointControl = ControlType.kMAXMotionPositionControl;
    private static final ClosedLoopSlot LOOP_SLOT = ClosedLoopSlot.kSlot0;
    private static final PIDConstants MOTOR_PID = new PIDConstants(0, 0, 0); // TODO: Figure PID Constants.
    private static final AngularVelocity MAX_LINEAR_VELOCITY = RPM.of(0); // TODO: Figure Max RPM
    private static final AngularAcceleration MAX_LINEAR_ACCELERATION = RotationsPerSecondPerSecond.of(0); // TODO:
                                                                                                          // Figure Max
                                                                                                          // Angular
                                                                                                          // Acceleration.
    private static final Angle ERROR_ROOM = Degrees.of(0); // TODO: Figure room of error. Account for conversion factor
                                                           // if applied.
    private static final MAXMotionConfig CLOSED_LOOP_MAX_MOTION_CONFIGURATION = new MAXMotionConfig()
            .cruiseVelocity(MAX_LINEAR_VELOCITY.in(RPM))
            .maxAcceleration(MAX_LINEAR_ACCELERATION.times(60).in(RotationsPerSecondPerSecond)) // TODO: Use RPMM
                                                                                                // Rotations Per Minute
                                                                                                // Per Minute. Instead
                                                                                                // of unknown number
                                                                                                // conversion.
            .allowedProfileError(ERROR_ROOM.in(Rotations));

    private static final Behavior REVERSE_LIMIT_BEHAVIOR = Behavior.kStopMovingMotorAndSetPosition;
    private static final Behavior FORWARD_LIMIT_BEHAVIOR = Behavior.kStopMovingMotorAndSetPosition;

    public IntakeBayDoorMotor(String name, CanId canId) {
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
        motorConfiguration.limitSwitch.reverseLimitSwitchPosition(CLOSED_ANGLE.in(Rotations));

        motorConfiguration.limitSwitch.forwardLimitSwitchTriggerBehavior(FORWARD_LIMIT_BEHAVIOR);
        motorConfiguration.limitSwitch.forwardLimitSwitchPosition(OPENED_ANGLE.in(Rotations));

        motorConfiguration.closedLoop
                .pid(MOTOR_PID.kP, MOTOR_PID.kI, MOTOR_PID.kD);

        motorConfiguration.closedLoop.maxMotion
                .apply(CLOSED_LOOP_MAX_MOTION_CONFIGURATION);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle (Degrees)", this::getAngle, null);
    }

    /**
     * 
     * @return Degrees
     */
    public double getAngle() {
        return Rotations.of(motor.getEncoder().getPosition()).in(Degrees);
    }

    @Override
    public void setAngularPosition(Angle angle) {
        closedLoopController.setSetpoint(angle.in(Rotations), setPointControl);
    }

    public void setFollower(IntakeBayDoorMotor leadMotor) {
        motorConfiguration.follow(leadMotor.canId.Id());
    }

    public void setInverted(boolean inverted) {
        motorConfiguration.inverted(inverted);
    }
}

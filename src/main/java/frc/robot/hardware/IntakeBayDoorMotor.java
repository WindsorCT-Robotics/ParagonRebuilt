package frc.robot.hardware;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.robot.hardware.base.NeoMotorBase;
import frc.robot.interfaces.IAngularPositionMotor;

public class IntakeBayDoorMotor extends NeoMotorBase implements IAngularPositionMotor {
    private static final IdleMode IDLE_MODE = IdleMode.kBrake;
    private static final boolean INVERTED = false;
    private static final Current CURRENT_LIMIT = Amps.of(0); // TODO: Get the recommended current limit of this motor.
    private final SparkBaseConfig motorConfiguration = new SparkMaxConfig().idleMode(IDLE_MODE).inverted(INVERTED);
    // https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-position-control#tuning-for-maxmotion-position-control
    private static final PIDConstants MOTOR_PID = new PIDConstants(0, 0, 0); // TODO: Test pid constants.
    private static final Angle CLOSED_LOOP_ERROR = Degrees.of(0.0); // TODO: Decide room of error.
    private static final ClosedLoopSlot LOOP_SLOT = ClosedLoopSlot.kSlot0;
    private final ClosedLoopConfig motorClosedLoopConfiguration = motorConfiguration.closedLoop
            .pid(MOTOR_PID.kP, MOTOR_PID.kI, MOTOR_PID.kD)
            .allowedClosedLoopError(0, LOOP_SLOT);

    private static final Angle CLOSED_ANGLE = Degrees.of(0);
    private static final Angle OPENED_ANGLE = Degrees.of(0); // TODO: How much should the motor turn to become open to
                                                             // intake fuel?

    // https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started#setting-up-closed-loop-control
    private final SparkClosedLoopController closedLoopController;
    private static final ControlType setPointControl = ControlType.kMAXMotionPositionControl;

    public IntakeBayDoorMotor(String name, CanId canId) {
        super(
                name,
                canId,
                new SparkMaxConfig()
                        .idleMode(IDLE_MODE)
                        .inverted(INVERTED)
                        .smartCurrentLimit((int) CURRENT_LIMIT.in(Amps)),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        closedLoopController = motor.getClosedLoopController();
    }

    @Override
    public Angle getAngle() {
        return Rotations.of(motor.getEncoder().getPosition());
    }

    @Override
    public void setAngularPosition(Angle angle) {
        closedLoopController.setSetpoint(angle.in(Rotations), setPointControl);
    }
}

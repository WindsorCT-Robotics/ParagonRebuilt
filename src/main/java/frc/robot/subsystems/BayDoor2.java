package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;

import java.util.HexFormat;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.hardware.CanId;
import frc.robot.hardware.DigitalInputOutput;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorMotorBasic;
import frc.robot.interfaces.ISystemDynamics;

public class BayDoor2 extends SubsystemBase implements ISystemDynamics<BayDoorMotorBasic> {
    private final BayDoorMotorBasic leftMotor;
    private final BayDoorMotorBasic rightMotor;
    private final DigitalInput leftHardLimit;
    private final DigitalInput rightHardLimit;

    private static final Dimensionless HOME_DUTY_CYCLE = Percent.of(-0.1);
    private static final Angle OPEN_ANGLE = Rotations.of(21.5);
    private static final Angle CLOSE_ANGLE = Rotations.of(0);
    private static final boolean INVERTED = true;
    // TODO: Configure these values.
    private static final FeedForwardConfig FEED_FORWARD_CONFIG = new FeedForwardConfig()
            .kS(0)
            .kG(0)
            .kV(0)
            .kA(0);
    private static final MAXMotionConfig MAX_MOTION_CONFIG = new MAXMotionConfig()
            .allowedProfileError(0)
            .cruiseVelocity(0)
            .maxAcceleration(0)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    private static final SoftLimitConfig SOFT_LIMIT_CONFIG = new SoftLimitConfig()
            .forwardSoftLimit(OPEN_ANGLE.in(Rotations))
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(CLOSE_ANGLE.in(Rotations))
            .reverseSoftLimitEnabled(true);

    public final Trigger atLeftCloseLimit;
    public final Trigger atRightCloseLimit;
    public final Trigger atLeftOpenLimit;
    public final Trigger atRightOpenLimit;
    public final Trigger isBayDoorClosed;
    public final Trigger isBayDoorOpen;

    public BayDoor2(
            String name,
            CanId leftMotorId,
            CanId rightMotorId,
            DigitalInputOutput leftLimitSwitchDIO,
            DigitalInputOutput rightLimitSwitchDIO) {
        super("Subsystems/" + name);

        leftHardLimit = new DigitalInput(leftLimitSwitchDIO.Id());
        rightHardLimit = new DigitalInput(rightLimitSwitchDIO.Id());
        leftMotor = new BayDoorMotorBasic("Left Motor", leftMotorId, leftHardLimit, this::setDutyCycle,
                this::setVoltage);
        rightMotor = new BayDoorMotorBasic("Right Motor", rightMotorId, rightHardLimit, this::setDutyCycle,
                this::setVoltage);

        // TODO: Be able to apply configuration and keep the same ResetMode and
        // PersisMode without hard coding.
        leftMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig().inverted(INVERTED);
            config.closedLoop.feedForward.apply(FEED_FORWARD_CONFIG);
            config.closedLoop.maxMotion.apply(MAX_MOTION_CONFIG);
            config.softLimit.apply(SOFT_LIMIT_CONFIG);

            motor.configure(config,
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kPersistParameters);
        });

        rightMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig().inverted(!INVERTED);
            config.closedLoop.feedForward.apply(FEED_FORWARD_CONFIG);
            config.closedLoop.maxMotion.apply(MAX_MOTION_CONFIG);
            config.softLimit.apply(SOFT_LIMIT_CONFIG);

            motor.configure(config,
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kPersistParameters);
        });

        atLeftCloseLimit = new Trigger(() -> new DigitalInput(leftLimitSwitchDIO.Id()).get());
        atRightCloseLimit = new Trigger(() -> new DigitalInput(rightLimitSwitchDIO.Id()).get());
        atLeftOpenLimit = new Trigger(() -> leftMotor.getAngle().gte(OPEN_ANGLE));
        atRightOpenLimit = new Trigger(() -> rightMotor.getAngle().gte(OPEN_ANGLE));
        isBayDoorClosed = new Trigger(atLeftCloseLimit.and(atRightCloseLimit));
        isBayDoorOpen = new Trigger(atLeftOpenLimit.and(atRightOpenLimit));
    }

    public Command home() {
        return runEnd(
                () -> {
                    leftMotor.home(HOME_DUTY_CYCLE);
                    rightMotor.home(HOME_DUTY_CYCLE);
                }, () -> removeDefaultCommand()).until(isBayDoorClosed)
                .handleInterrupt(() -> setDefaultCommand(home()));
    }

    public Command open() {
        return runOnce(() -> );
    }

    @Override
    public void log(SysIdRoutineLog log, BayDoorMotorBasic motor, String name) {
        // TODO Auto-generated method stub

    }

    @Override
    public Command sysIdDynamic(Direction direction) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Command sysIdQuasistatic(Direction direction) {
        // TODO Auto-generated method stub
        return null;
    }
}

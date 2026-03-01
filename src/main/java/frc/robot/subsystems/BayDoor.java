package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.hardware.CanId;
import frc.robot.hardware.DigitalInputOutput;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorMotorBasic;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorState;
import frc.robot.interfaces.ISystemDynamics;

public class BayDoor extends SubsystemBase implements ISystemDynamics<BayDoorMotorBasic> {
    private final BayDoorMotorBasic leftMotor;
    private final BayDoorMotorBasic rightMotor;
    private final DigitalInput leftHardLimit;
    private final DigitalInput rightHardLimit;

    private static final Dimensionless HOME_DUTY_CYCLE = Percent.of(-0.3);
    private static final Dimensionless DUTY_CYCLE = Percent.of(0.5);
    private static final Angle OPEN_ANGLE = Rotations.of(20);
    private static final Angle CLOSE_ANGLE = Rotations.of(0);
    private static final boolean INVERTED = true;
    // TODO: Configure these values.
    private static final FeedForwardConfig FEED_FORWARD_CONFIG = new FeedForwardConfig()
            .kS(0.3)
            .kCos(0)
            .kV(0.00025)
            .kA(0);

    private static final ClosedLoopConfig CLOSED_LOOP_CONFIG = new ClosedLoopConfig()
            .p(1)
            .i(0)
            .d(0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    private static final MAXMotionConfig MAX_MOTION_CONFIG = new MAXMotionConfig()
            .allowedProfileError(1)
            .cruiseVelocity(3000)
            .maxAcceleration(1000)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    private static final SoftLimitConfig SOFT_LIMIT_CONFIG = new SoftLimitConfig()
            .forwardSoftLimit(OPEN_ANGLE.in(Rotations))
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(CLOSE_ANGLE.in(Rotations))
            .reverseSoftLimitEnabled(true);

    private final SysIdRoutine routine;

    public final Trigger atLeftCloseLimit;
    public final Trigger atRightCloseLimit;
    public final Trigger atLeftSoftCloseLimit;
    public final Trigger atRightSoftCloseLimit;
    public final Trigger atLeftOpenLimit;
    public final Trigger atRightOpenLimit;
    public final Trigger isBayDoorClosed;
    public final Trigger isBayDoorSoftClosed;
    public final Trigger isBayDoorOpen;

    private static final ResetMode RESET_MODE = ResetMode.kNoResetSafeParameters;
    private static final PersistMode PERSIST_MODE = PersistMode.kPersistParameters;

    private static final Angle TOLERANCE = Rotations.of(1);

    public BayDoor(
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

        routine = new SysIdRoutine(
                new Config(
                        Volts.of(1).per(Second),
                        Volts.of(1),
                        null),
                new Mechanism(this::setSysIdVoltage, log -> {
                    log(log, leftMotor, "Left Motor");
                    log(log, rightMotor, "Right Motor");
                }, this));

        leftMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig().inverted(INVERTED);
            config.closedLoop.feedForward.apply(FEED_FORWARD_CONFIG);
            config.closedLoop.apply(CLOSED_LOOP_CONFIG);
            config.closedLoop.maxMotion.apply(MAX_MOTION_CONFIG);
            config.softLimit.apply(SOFT_LIMIT_CONFIG);

            motor.configure(config,
                    RESET_MODE,
                    PERSIST_MODE);
        });

        rightMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig().inverted(!INVERTED);
            config.closedLoop.feedForward.apply(FEED_FORWARD_CONFIG);
            config.closedLoop.apply(CLOSED_LOOP_CONFIG);
            config.closedLoop.maxMotion.apply(MAX_MOTION_CONFIG);
            config.softLimit.apply(SOFT_LIMIT_CONFIG);

            motor.configure(config,
                    RESET_MODE,
                    PERSIST_MODE);
        });

        // rightMotor.follow(leftMotor.getId(), true);

        atLeftCloseLimit = new Trigger(() -> leftHardLimit.get());
        atRightCloseLimit = new Trigger(() -> rightHardLimit.get());
        atLeftSoftCloseLimit = new Trigger(() -> leftMotor.getAngle().isEquivalent(CLOSE_ANGLE));
        atRightSoftCloseLimit = new Trigger(() -> rightMotor.getAngle().isEquivalent(CLOSE_ANGLE));
        atLeftOpenLimit = new Trigger(() -> leftMotor.getAngle().gte(OPEN_ANGLE));
        atRightOpenLimit = new Trigger(() -> rightMotor.getAngle().gte(OPEN_ANGLE));
        isBayDoorClosed = new Trigger(atLeftCloseLimit.and(atRightCloseLimit));
        isBayDoorSoftClosed = new Trigger(atLeftSoftCloseLimit.and(atRightSoftCloseLimit));
        isBayDoorOpen = new Trigger(atLeftOpenLimit.and(atRightOpenLimit));

        initSmartDashboard();
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/Left Limit Switch", leftHardLimit);
        SmartDashboard.putData(getName() + "/Right Limit Switch", rightHardLimit);
        SmartDashboard.putData(getName() + "/" + leftMotor.getSmartDashboardName(), leftMotor);
        SmartDashboard.putData(getName() + "/" + rightMotor.getSmartDashboardName(), rightMotor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Is Intake Closed?", isBayDoorClosed, null);
        builder.addBooleanProperty("Is Intake Open?", isBayDoorOpen, null);
        builder.addBooleanProperty("Is Left Pressed", () -> leftMotor.isHomed(), null);
        builder.addBooleanProperty("Is Right Pressed", () -> rightMotor.isHomed(), null);
    }

    private void moveTowards(BayDoorMotorBasic motor, Dimensionless percent, Angle goalAngle, Angle currentAngle) {
        if (currentAngle.lte(goalAngle))
            motor.setDutyCycle(percent);
            motor.setBayMotorState(BayDoorState.OPENING);
        if (currentAngle.gt(goalAngle))
            motor.setDutyCycle(percent.unaryMinus());
            motor.setBayMotorState(BayDoorState.CLOSING);
    }

    private void moveToPosition(
        BayDoorMotorBasic motor, 
        Dimensionless percent, 
        Angle goalAngle, 
        Angle currentAngle,
        BayDoorState endState) {
            if (!currentAngle.isNear(goalAngle, TOLERANCE)) {
                moveTowards(motor, percent, goalAngle, currentAngle);
            } else {
                motor.stop();
                motor.setBayMotorState(endState);
            }
        }

    public Command home() {
        return runEnd(
                () -> {
                    leftMotor.home(HOME_DUTY_CYCLE);
                    rightMotor.home(HOME_DUTY_CYCLE);
                }, () -> removeDefaultCommand()).until(isBayDoorClosed)
                .handleInterrupt(() -> setDefaultCommand(home())).withName("Home");
    }

    public Command open() {
        return runEnd(() -> {
            moveToPosition(leftMotor, DUTY_CYCLE, OPEN_ANGLE, leftMotor.getAngle(), BayDoorState.OPEN);
            moveToPosition(rightMotor, DUTY_CYCLE, OPEN_ANGLE, rightMotor.getAngle(), BayDoorState.OPEN);
        }, this::stop).withName("Open");
    }

    public Command close() {
        return runEnd(() -> {
            moveToPosition(leftMotor, DUTY_CYCLE, CLOSE_ANGLE, leftMotor.getAngle(), BayDoorState.CLOSE);
            moveToPosition(rightMotor, DUTY_CYCLE, CLOSE_ANGLE, rightMotor.getAngle(), BayDoorState.CLOSE);
        }, this::home).until(isBayDoorClosed.or(isBayDoorSoftClosed)).withName("Close");
    }

    // region SysId
    @Override
    public void log(SysIdRoutineLog log, BayDoorMotorBasic motor, String name) {
        log.motor(
                name)
                .angularPosition(motor.getAngle())
                .angularVelocity(motor.getVelocity())
                .voltage(motor.getVoltage());
    }

    @Override
    public Command sysIdDynamic(Direction direction) {
        return routine.dynamic(direction).withName(getSubsystem() + "/sysIdDynamic");
    }

    @Override
    public Command sysIdQuasistatic(Direction direction) {
        return routine.quasistatic(direction).withName(getSubsystem() + "/sysIdQuasistatic");
    }

    private void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }

    private void setDutyCycle(Dimensionless dutyCycle) {
        CommandScheduler.getInstance().schedule(overrideMotorDutyCycle(dutyCycle));
    }

    private void setVoltage(Voltage voltage) {
        CommandScheduler.getInstance().schedule(overrideMotorVoltage(voltage));
    }

    private void setSysIdVoltage(Voltage voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    public Command overrideMotorDutyCycle(Dimensionless dutyCycle) {
        return runEnd(() -> {
            leftMotor.setDutyCycle(dutyCycle);
            rightMotor.setDutyCycle(dutyCycle);
        }, this::stop).withName(getSubsystem() + "/overrideMotorDutyCycle");
    }

    public Command overrideMotorVoltage(Voltage voltage) {
        return runEnd(() -> {
            leftMotor.setVoltage(voltage);
            rightMotor.setVoltage(voltage);
        }, this::stop).withName(getSubsystem() + "/overrideMotorVoltage");
    }
    // endregion
}

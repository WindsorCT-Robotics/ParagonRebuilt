package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
    private final ArmFeedforward ff;
    private static final Angle FORWARD_POSITION_TOLERANCE = Rotations.of(1);
    private static final Angle REVERSE_POSITION_TOLERANCE = Rotations.of(1);
    private static final boolean INVERTED = true;
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(1);
    private static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(1);
    private static final TrapezoidProfile.Constraints MOTION_CONSTRAINTS = new Constraints(
            MAX_ANGULAR_VELOCITY.in(RotationsPerSecond), MAX_ANGULAR_ACCELERATION.in(RotationsPerSecondPerSecond));
    public static final Angle OPEN_ANGLE = Rotations.of(21.5);
    public static final Angle CLOSE_ANGLE = Rotations.of(0);

    public final Trigger isAtRightLimit;
    public final Trigger isAtLeftLimit;
    public final Trigger isIntakeClosed;
    public final Trigger isIntakeOpen;

    private final SysIdRoutine routine;

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
                this::setVelocity,
                this::setVoltage);
        rightMotor = new BayDoorMotorBasic("Right Motor", rightMotorId, rightHardLimit, this::setDutyCycle,
                this::setVelocity, this::setVoltage);

        // TODO: Be able to apply configuration and keep the same ResetMode and
        // PersisMode without hard coding.
        leftMotor.configure(motor -> {
            motor.configure(new SparkMaxConfig().inverted(INVERTED), ResetMode.kNoResetSafeParameters,
                    PersistMode.kPersistParameters);
        });

        rightMotor.configure(motor -> {
            motor.configure(new SparkMaxConfig().inverted(!INVERTED), ResetMode.kNoResetSafeParameters,
                    PersistMode.kPersistParameters);
        });

        isAtLeftLimit = new Trigger(leftHardLimit::get);
        isAtRightLimit = new Trigger(rightHardLimit::get);

        isIntakeClosed = new Trigger(
                () -> leftMotor.getAngle().lte(CLOSE_ANGLE) && rightMotor.getAngle().lte(CLOSE_ANGLE));
        isIntakeOpen = new Trigger(() -> leftMotor.getAngle().gte(OPEN_ANGLE) && rightMotor.getAngle().gte(OPEN_ANGLE));

        ff = new ArmFeedforward(1.3 / 100, 0.5 / 100, ((1.3 / 100) / 0.6));

        // TODO: Consider customizing new Config(). Should be customized if motor has
        // physical limitations.
        routine = new SysIdRoutine(new Config(), new Mechanism(this::setVoltage, log -> {
            log(log, leftMotor, "Left Motor");
            log(log, rightMotor, name);
        }, this));

        addChild(getName(), leftHardLimit);
        addChild(getName(), rightHardLimit);
        addChild(getName(), leftMotor);
        addChild(getName(), rightMotor);

        initSmartDashboard();
    }

    private enum BayDoorAction {
        OPEN,
        CLOSE
    }

    public Command home() {
        return run(() -> {
            leftMotor.home();
            rightMotor.home();
        })
                .until(() -> leftMotor.isHomed() && rightMotor.isHomed())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName(getSubsystem() + "/home");
    }

    public Command openBayDoor() {
        return moveBayDoorTo(BayDoorAction.OPEN);
    }

    public Command closeBayDoor() {
        return moveBayDoorTo(BayDoorAction.CLOSE);
    }

    private void moveTowards(
            BayDoorMotorBasic motor,
            Angle position,
            TrapezoidProfile.Constraints constraints) {
        TrapezoidProfile.State currentState = new State(motor.getAngle().in(Rotations),
                motor.getVelocity().in(RotationsPerSecond));
        TrapezoidProfile.State goalState = new State(position.in(Rotations),
                RotationsPerSecond.zero().in(RotationsPerSecond));
        TrapezoidProfile motionProfile = new TrapezoidProfile(constraints);

        AngularVelocity velocity = RotationsPerSecond
                .of(motionProfile.calculate(TimedRobot.kDefaultPeriod, currentState, goalState).velocity);

        motor.setVelocity(velocity);
        // TODO: Remove these or put them somewhere else after testing.
        SmartDashboard.putNumber("Desired Position", goalState.position);
        SmartDashboard.putNumber("Velocity", velocity.in(RotationsPerSecond));
    }

    private void moveToPosition(
            BayDoorMotorBasic motor,
            Angle position,
            Angle goalPosition,
            Angle tolerance,
            BayDoorState medianState,
            BayDoorState endState) {
        if (!position.isNear(goalPosition, tolerance)) {
            moveTowards(motor, goalPosition, MOTION_CONSTRAINTS);
            motor.setBayMotorState(medianState);
        } else {
            motor.stop();
            motor.setBayMotorState(endState);
        }
    }

    private Command moveBayDoorTo(BayDoorAction action) {
        Angle goalPosition;
        Angle tolerance;
        BayDoorState medianState;
        BayDoorState endState;

        switch (action) {
            case OPEN:
                goalPosition = OPEN_ANGLE;
                leftMotor.setBayMotorState(BayDoorState.OPENING);
                rightMotor.setBayMotorState(BayDoorState.OPENING);
                tolerance = FORWARD_POSITION_TOLERANCE;
                medianState = BayDoorState.OPENING;
                endState = BayDoorState.OPEN;
                break;
            case CLOSE:
                goalPosition = CLOSE_ANGLE;
                leftMotor.setBayMotorState(BayDoorState.CLOSING);
                rightMotor.setBayMotorState(BayDoorState.CLOSING);
                tolerance = REVERSE_POSITION_TOLERANCE;
                medianState = BayDoorState.CLOSING;
                endState = BayDoorState.CLOSE;
                break;
            default:
                throw new IllegalStateException("Unknown Bay Door Action: " + action);
        }

        return run(() -> {
            moveToPosition(leftMotor, leftMotor.getAngle(), goalPosition, tolerance, medianState, endState);
            moveToPosition(rightMotor, rightMotor.getAngle(), goalPosition, tolerance, medianState, endState);
        }).withName(getSubsystem() + "/moveBayDoorTo");
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/Left Bay Door Limit Switch", leftHardLimit);
        SmartDashboard.putData(getName() + "/Right Bay Door Limit Switch", rightHardLimit);
        SmartDashboard.putData(getName() + "/Left " + leftMotor.getClass().getSimpleName(), leftMotor);
        SmartDashboard.putData(getName() + "/Right " + rightMotor.getClass().getSimpleName(), rightMotor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Feed Forward Static Gain (V)", ff::getKs, ff::setKs);
        builder.addDoubleProperty("Feed Forward Gravity Gain (V)", ff::getKg, ff::setKg);
        builder.addDoubleProperty("Feed Forward Velocity Gain (V/(rad/s))", ff::getKv, ff::setKv);
        builder.addDoubleProperty("Feed Forward Acceleration Gain (V/(rad/s^2))", ff::getKa, ff::setKa);
        builder.addBooleanProperty("Is Intake Closed?", isIntakeClosed, null);
        builder.addBooleanProperty("Is Intake Open?", isIntakeOpen, null);
    }

    private void setDutyCycle(Dimensionless dutyCycle) {
        CommandScheduler.getInstance().schedule(overrideMotorDutyCycle(dutyCycle));
    }

    private void setVelocity(AngularVelocity velocity) {
        CommandScheduler.getInstance().schedule(overrideMotorVelocity(velocity));
    }

    private void setVoltage(Voltage voltage) {
        CommandScheduler.getInstance().schedule(overrideMotorVoltage(voltage));
    }

    private void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }

    public Command overrideMotorDutyCycle(Dimensionless dutyCycle) {
        return runEnd(() -> {
            leftMotor.setDutyCycle(dutyCycle);
            rightMotor.setDutyCycle(dutyCycle);
        }, this::stop).withName(getSubsystem() + "/overrideMotorDutyCycle");
    }

    public Command overrideMotorVelocity(AngularVelocity velocity) {
        return runEnd(() -> {
            leftMotor.setVelocity(velocity);
            rightMotor.setVelocity(velocity);
        }, this::stop).withName(getSubsystem() + "/overrideMotorVelocity");
    }

    public Command overrideMotorVoltage(Voltage voltage) {
        return runEnd(() -> {
            leftMotor.setVoltage(voltage);
            rightMotor.setVoltage(voltage);
        }, this::stop).withName(getSubsystem() + "/overrideMotorVoltage");
    }

    @Override
    public void log(SysIdRoutineLog log, BayDoorMotorBasic motor, String name) {
        log.motor(
                name)
                .angularPosition(motor.getAngle())
                .angularVelocity(motor.getVelocity());
    }

    @Override
    public Command sysIdDynamic(Direction direction) {
        return routine.dynamic(direction).withName(getSubsystem() + "/sysIdDynamic");
    }

    @Override
    public Command sysIdQuasistatic(Direction direction) {
        return routine.quasistatic(direction).withName(getSubsystem() + "/sysIdQuasistatic");
    }
}

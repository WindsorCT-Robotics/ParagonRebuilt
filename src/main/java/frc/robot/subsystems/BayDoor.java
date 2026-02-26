package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.FeedForwardConfigAccessor;
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
import frc.robot.hardware.base_motors.SparkMaxMotorBase;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorMotorBasic;
import frc.robot.interfaces.ISystemDynamics;

public class BayDoor extends SubsystemBase implements ISystemDynamics<BayDoorMotorBasic> {
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
            .kCos(0)
            .kV(0) // TODO: What should kV be?
            .kA(0);

    private static final ClosedLoopConfig CLOSED_LOOP_CONFIG = new ClosedLoopConfig()
            .p(0)
            .i(0)
            .d(0);

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

    private final SysIdRoutine routine;

    public final Trigger atLeftCloseLimit;
    public final Trigger atRightCloseLimit;
    public final Trigger atLeftOpenLimit;
    public final Trigger atRightOpenLimit;
    public final Trigger isBayDoorClosed;
    public final Trigger isBayDoorOpen;

    private static final ResetMode RESET_MODE = ResetMode.kNoResetSafeParameters;
    private static final PersistMode PERSIST_MODE = PersistMode.kPersistParameters;

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

        // TODO: Be able to apply configuration and keep the same ResetMode and
        // PersisMode without hard coding.
        leftMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig().inverted(INVERTED);
            config.closedLoop.feedForward.apply(FEED_FORWARD_CONFIG);
            config.closedLoop.apply(CLOSED_LOOP_CONFIG);
            config.closedLoop.maxMotion.apply(MAX_MOTION_CONFIG);
            config.softLimit.apply(SOFT_LIMIT_CONFIG);

            motor.configure(config,
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kPersistParameters);
        });

        rightMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig().inverted(!INVERTED);
            config.closedLoop.feedForward.apply(FEED_FORWARD_CONFIG);
            config.closedLoop.apply(CLOSED_LOOP_CONFIG);
            config.closedLoop.maxMotion.apply(MAX_MOTION_CONFIG);
            config.softLimit.apply(SOFT_LIMIT_CONFIG);

            motor.configure(config,
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kPersistParameters);
        });

        atLeftCloseLimit = new Trigger(() -> leftHardLimit.get());
        atRightCloseLimit = new Trigger(() -> rightHardLimit.get());
        atLeftOpenLimit = new Trigger(() -> leftMotor.getAngle().gte(OPEN_ANGLE));
        atRightOpenLimit = new Trigger(() -> rightMotor.getAngle().gte(OPEN_ANGLE));
        isBayDoorClosed = new Trigger(atLeftCloseLimit.and(atRightCloseLimit));
        isBayDoorOpen = new Trigger(atLeftOpenLimit.and(atRightOpenLimit));

        initSmartDashboard();
    }

    private void setkS(double value) {
        leftMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.feedForward.kS(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });

        rightMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.feedForward.kS(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });
    }

    private void setkCos(double value) {
        leftMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.feedForward.kCos(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });

        rightMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.feedForward.kCos(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });
    }

    private void setkG(double value) {
        leftMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.feedForward.kS(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });

        rightMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.feedForward.kS(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });
    }

    private void setkV(double value) {
        leftMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.feedForward.kV(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });

        rightMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.feedForward.kV(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });
    }

    private void setkA(double value) {
        leftMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.feedForward.kS(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });

        rightMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.feedForward.kS(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });
    }

    private FeedForwardConfigAccessor getFeedForward(SparkMaxMotorBase motor) {
        return motor.getConfiguration().closedLoop.feedForward;
    }

    private void setP(double value) {
        leftMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.p(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });

        rightMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.p(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });
    }

    private void setI(double value) {
        leftMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.i(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });

        rightMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.i(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });
    }

    private void setD(double value) {
        leftMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.d(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });

        rightMotor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig();
            config.closedLoop.d(value);
            motor.configure(config, RESET_MODE, PERSIST_MODE);
        });
    }

    private ClosedLoopConfigAccessor getPID(SparkMaxMotorBase motor) {
        return motor.getConfiguration().closedLoop;
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/Left Bay Door Limit Switch", leftHardLimit);
        SmartDashboard.putData(getName() + "/Right Bay Door Limit Switch", rightHardLimit);
        SmartDashboard.putData(getName() + "/Left ", leftMotor);
        SmartDashboard.putData(getName() + "/Right ", rightMotor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Is Intake Closed?", isBayDoorClosed, null);
        builder.addBooleanProperty("Is Intake Open?", isBayDoorOpen, null);
        builder.addBooleanProperty("Is Left Pressed", () -> leftMotor.isHomed(), null);
        builder.addBooleanProperty("Is Right Pressed", () -> rightMotor.isHomed(), null);
        builder.addDoubleProperty("Motors/kS", () -> getFeedForward(leftMotor).getkS(), this::setkS);
        builder.addDoubleProperty("Motors/kCos", () -> getFeedForward(leftMotor).getkCos(), this::setkCos);
        builder.addDoubleProperty("Motors/kG", () -> getFeedForward(leftMotor).getkG(), this::setkG);
        builder.addDoubleProperty("Motors/kV", () -> getFeedForward(leftMotor).getkV(), this::setkV);
        builder.addDoubleProperty("Motors/kA", () -> getFeedForward(leftMotor).getkA(), this::setkA);
        builder.addDoubleProperty("Motors/P", () -> getPID(leftMotor).getP(), this::setP);
        builder.addDoubleProperty("Motors/I", () -> getPID(leftMotor).getI(), this::setI);
        builder.addDoubleProperty("Motors/D", () -> getPID(leftMotor).getD(), this::setD);
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
        return runOnce(() -> {
            leftMotor.setPointPosition(OPEN_ANGLE);
            rightMotor.setPointPosition(OPEN_ANGLE);
        });
    }

    public Command middle() {
        return runOnce(() -> {
            leftMotor.setPointPosition(OPEN_ANGLE.div(2));
            rightMotor.setPointPosition(OPEN_ANGLE.div(2));
        });
    }

    public Command close() {
        return runOnce(() -> {
            leftMotor.setPointPosition(CLOSE_ANGLE);
            rightMotor.setPointPosition(CLOSE_ANGLE);
        });
    }

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
}

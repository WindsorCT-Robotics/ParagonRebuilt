package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
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
    private final TalonFXConfiguration leftMotorConfiguration;
    private final TalonFXConfiguration rightMotorConfiguration;
    private final DigitalInput leftHardLimit;
    private final DigitalInput rightHardLimit;

    private static final Dimensionless HOME_DUTY_CYCLE = Percent.of(-0.1);
    private static final Dimensionless DUTY_CYCLE = Percent.of(0.2);
    private static final Angle OPEN_ANGLE = Rotations.of(5.85);
    private static final Angle CLOSE_ANGLE = Rotations.of(0);

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

    private static final Angle TOLERANCE = Rotations.of(1);

    public BayDoor(
            String name,
            CanId leftMotorId,
            CanId rightMotorId,
            DigitalInputOutput leftLimitSwitchDIO,
            DigitalInputOutput rightLimitSwitchDIO) {
        super("Subsystems/" + name);

        leftMotor = new BayDoorMotorBasic("Left Bay Door Motor", leftMotorId);
        rightMotor = new BayDoorMotorBasic("Right Bay Door Motor", rightMotorId);

        leftMotorConfiguration = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitThreshold(OPEN_ANGLE)
                        .withReverseSoftLimitThreshold(CLOSE_ANGLE)
                        .withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitEnable(true))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(25))
                        .withSupplyCurrentLimit(Amps.of(20))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimitEnable(true))
                .withTorqueCurrent(new TorqueCurrentConfigs()
                        .withPeakForwardTorqueCurrent(Amps.of(10))
                        .withPeakReverseTorqueCurrent(Amps.of(10)));

        rightMotorConfiguration = leftMotorConfiguration.clone()
                .withMotorOutput(leftMotorConfiguration.clone().MotorOutput
                        .withInverted((leftMotorConfiguration.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
                                ? InvertedValue.CounterClockwise_Positive
                                : InvertedValue.Clockwise_Positive));

        leftHardLimit = new DigitalInput(leftLimitSwitchDIO.Id());
        rightHardLimit = new DigitalInput(rightLimitSwitchDIO.Id());

        leftMotor.configure(configurator -> {
            configurator.apply(leftMotorConfiguration);
        });

        rightMotor.configure(configurator -> {
            configurator.apply(rightMotorConfiguration);
        });

        routine = new SysIdRoutine(
                new Config(
                        Volts.of(1).per(Second),
                        Volts.of(1),
                        null),
                new Mechanism(this::setSysIdVoltage, log -> {
                    log(log, leftMotor, "Left Motor");
                    log(log, rightMotor, "Right Motor");
                }, this));

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
        builder.addBooleanProperty("Is Left Pressed", atLeftCloseLimit, null);
        builder.addBooleanProperty("Is Right Pressed", atRightCloseLimit, null);
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

    private void enableSoftLimits(boolean enable) {
        leftMotor.configure(configuration -> {
            configuration.apply(
                    leftMotorConfiguration.clone().withSoftwareLimitSwitch(
                            leftMotorConfiguration.clone().SoftwareLimitSwitch
                                    .withForwardSoftLimitEnable(enable)
                                    .withReverseSoftLimitEnable(enable)));
        });

        rightMotor.configure(configuration -> {
            configuration.apply(
                    rightMotorConfiguration.clone().withSoftwareLimitSwitch(
                            rightMotorConfiguration.clone().SoftwareLimitSwitch
                                    .withForwardSoftLimitEnable(enable)
                                    .withReverseSoftLimitEnable(enable)));
        });
    }

    public Command home() {
        enableSoftLimits(false);
        return runEnd(
                () -> {
                    if (!atLeftCloseLimit.getAsBoolean()) {
                        leftMotor.setDutyCycle(HOME_DUTY_CYCLE);
                        leftMotor.setBayMotorState(BayDoorState.CLOSING);
                    } else {
                        leftMotor.stop();
                    }

                    if (!atRightCloseLimit.getAsBoolean()) {
                        rightMotor.setDutyCycle(HOME_DUTY_CYCLE);
                        rightMotor.setBayMotorState(BayDoorState.CLOSING);
                    } else {
                        rightMotor.stop();
                    }
                }, () -> {
                    stop();
                    leftMotor.resetRelativeEncoder();
                    rightMotor.resetRelativeEncoder();
                    enableSoftLimits(true);
                    removeDefaultCommand();
                }).until(isBayDoorClosed)
                .handleInterrupt(() -> setDefaultCommand(home()))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("Home");
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

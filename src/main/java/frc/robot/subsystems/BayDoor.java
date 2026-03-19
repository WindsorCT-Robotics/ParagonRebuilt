package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.generated.Elastic;
import frc.robot.generated.Elastic.Notification;
import frc.robot.hardware.BayMotorState;
import frc.robot.hardware.CanId;
import frc.robot.hardware.DigitalInputOutput;
import frc.robot.hardware.motors.BayDoorMotor;
import frc.robot.interfaces.ISystemDynamics;

public class BayDoor extends SubsystemBase implements ISystemDynamics<BayDoorMotor> {
    /**
     * Left Motor
     */
    private final BayDoorMotor leftMotor;
    /**
     * Right Motor
     */
    private final BayDoorMotor rightMotor;

    private final DigitalInput leftHardLimit;
    private final DigitalInput rightHardLimit;

    private final Elastic.Notification homeNotification;
    private final SysIdRoutine routine;

    private static final Dimensionless HOME_DUTY_CYCLE = Percent.of(-15);
    private static final Angle OPEN_ANGLE = Rotations.of(5.85);
    private static final Angle CLOSE_ANGLE = Rotations.of(0);

    private static final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(OPEN_ANGLE.plus(Rotations.of(0.4)))
            .withReverseSoftLimitThreshold(CLOSE_ANGLE)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true);
    private static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(25));
    private static final TorqueCurrentConfigs torqueCurrentConfigs = new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(Amps.of(10))
            .withPeakReverseTorqueCurrent(Amps.of(10));

    private static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
    .withGravityType(GravityTypeValue.Arm_Cosine)
    .withKG(0.1)
    .withKS(0.05)
    .withKP(0.07);

    public final Trigger atLeftCloseLimit;
    public final Trigger atRightCloseLimit;
    public final Trigger atLeftSoftCloseLimit;
    public final Trigger atRightSoftCloseLimit;
    public final Trigger atLeftSoftOpenLimit;
    public final Trigger atRightSoftOpenLimit;
    public final Trigger isBayDoorClosed;
    public final Trigger isBayDoorSoftClosed;
    public final Trigger isBayDoorSoftOpen;

    public BayDoor(
            String name,
            CanId leftMotorId,
            CanId rightMotorId,
            DigitalInputOutput leftLimitSwitchDIO,
            DigitalInputOutput rightLimitSwitchDIO) {
        super("Subsystems/" + name);

        homeNotification = new Notification(Elastic.NotificationLevel.INFO, name + " has been HOMED", "");

        leftMotor = new BayDoorMotor("Left Bay Door Motor", leftMotorId, new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(SLOT0_CONFIGS)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
                .withCurrentLimits(currentLimitsConfigs)
                .withTorqueCurrent(torqueCurrentConfigs));
        rightMotor = new BayDoorMotor("Right Bay Door Motor", rightMotorId, new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(SLOT0_CONFIGS)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
                .withCurrentLimits(currentLimitsConfigs)
                .withTorqueCurrent(torqueCurrentConfigs));

        leftHardLimit = new DigitalInput(leftLimitSwitchDIO.Id());
        rightHardLimit = new DigitalInput(rightLimitSwitchDIO.Id());

        routine = new SysIdRoutine(
                new Config(
                        Volts.of(1).per(Second),
                        Volts.of(1),
                        null),
                new Mechanism(this::setSysIdVoltage, log -> {
                    log(log, leftMotor, "Left Motor");
                    log(log, rightMotor, "Right Motor");
                }, this));

        atLeftCloseLimit = new Trigger(() -> leftHardLimit.get())
                .onTrue(new InstantCommand(() -> leftMotor.setBayMotorState(BayMotorState.CLOSE)));

        atRightCloseLimit = new Trigger(() -> rightHardLimit.get())
                .onTrue(new InstantCommand(() -> rightMotor.setBayMotorState(BayMotorState.CLOSE)));

        atLeftSoftCloseLimit = new Trigger(() -> leftMotor.getAngle().lt(CLOSE_ANGLE))
                .onTrue(new InstantCommand(() -> leftMotor.setBayMotorState(BayMotorState.CLOSE)));

        atRightSoftCloseLimit = new Trigger(() -> rightMotor.getAngle().lt(CLOSE_ANGLE))
                .onTrue(new InstantCommand(() -> rightMotor.setBayMotorState(BayMotorState.CLOSE)));

        atLeftSoftOpenLimit = new Trigger(() -> leftMotor.getAngle().gte(OPEN_ANGLE))
                .onTrue(new InstantCommand(() -> leftMotor.setBayMotorState(BayMotorState.CLOSE)));

        atRightSoftOpenLimit = new Trigger(() -> rightMotor.getAngle().gte(OPEN_ANGLE))
                .onTrue(new InstantCommand(() -> rightMotor.setBayMotorState(BayMotorState.CLOSE)));

        isBayDoorClosed = new Trigger(atLeftCloseLimit.and(atRightCloseLimit));
        isBayDoorSoftClosed = new Trigger(atLeftSoftCloseLimit.and(atRightSoftCloseLimit));
        isBayDoorSoftOpen = new Trigger(atLeftSoftOpenLimit.and(atRightSoftOpenLimit));

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

        builder.addBooleanProperty("atLeftCloseLimit", atLeftCloseLimit, null);
        builder.addBooleanProperty("atRightCloseLimit", atRightCloseLimit, null);

        builder.addBooleanProperty("atLeftSoftCloseLimit", atLeftSoftCloseLimit, null);
        builder.addBooleanProperty("atRightSoftCloseLimit", atRightSoftCloseLimit, null);

        builder.addBooleanProperty("atLeftSoftOpenLimit", atLeftSoftOpenLimit, null);
        builder.addBooleanProperty("atRightSoftOpenLimit", atRightSoftOpenLimit, null);

        builder.addBooleanProperty("isBayDoorClosed", isBayDoorClosed, null);
        builder.addBooleanProperty("isBayDoorSoftClosed", isBayDoorSoftClosed, null);
        builder.addBooleanProperty("isBayDoorSoftOpen", isBayDoorSoftOpen, null);
    }

    private void enableSoftLimits(boolean enable) {
        TalonFXConfiguration leftMotorConfig = leftMotor.getCurrentConfiguration()
                .withSoftwareLimitSwitch(leftMotor.getCurrentConfiguration().SoftwareLimitSwitch
                        .withForwardSoftLimitEnable(enable).withReverseSoftLimitEnable(enable));
        TalonFXConfiguration rightMotorConfig = rightMotor.getCurrentConfiguration()
                .withSoftwareLimitSwitch(rightMotor.getCurrentConfiguration().SoftwareLimitSwitch
                        .withForwardSoftLimitEnable(enable).withReverseSoftLimitEnable(enable));

        leftMotor.configure(leftMotorConfig);
        rightMotor.configure(rightMotorConfig);
    }

    public Command home() {
        return runEnd(
                () -> {
                    leftMotor.home(atLeftCloseLimit.getAsBoolean(), HOME_DUTY_CYCLE);
                    rightMotor.home(atRightCloseLimit.getAsBoolean(), HOME_DUTY_CYCLE);
                }, this::onHomingComplete)
                .withName("Home")
                .beforeStarting(() -> enableSoftLimits(false))
                .until(isBayDoorClosed)
                .handleInterrupt(() -> setDefaultCommand(home()))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    private void onHomingComplete() {
        enableSoftLimits(true);
        removeDefaultCommand();
        Elastic.sendNotification(homeNotification);
    }

    public Command open() {
        return runEnd(() -> {
            leftMotor.setPointPosition(OPEN_ANGLE);
            rightMotor.setPointPosition(OPEN_ANGLE);
        }, this::stop).withName("Open");
    }

    public Command close() {
        return runEnd(() -> {
            leftMotor.setPointPosition(CLOSE_ANGLE);
            rightMotor.setPointPosition(CLOSE_ANGLE);
        }, this::stop).withName("Close");
    }

    // region SysId
    @Override
    public void log(SysIdRoutineLog log, BayDoorMotor motor, String name) {
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

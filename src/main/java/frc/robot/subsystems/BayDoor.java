package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.Elastic;
import frc.robot.generated.Elastic.Notification;
import frc.robot.hardware.CanId;
import frc.robot.hardware.motors.BayDoorMotor;
import frc.robot.hardware.sensors.BayDoorAbsoluteEncoder;

public class BayDoor extends SubsystemBase {
        private static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKG(0.1)
                        .withKS(0.05)
                        .withKP(0.07);

        private static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(25));

        private final BayDoorMotor leftMotor = new BayDoorMotor(
                        "Left Bay Door Motor",
                        new CanId((byte) 14),
                        new TalonFXConfiguration()
                                        .withMotorOutput(new MotorOutputConfigs()
                                                        .withInverted(InvertedValue.CounterClockwise_Positive)
                                                        .withNeutralMode(NeutralModeValue.Brake))
                                        .withSlot0(SLOT0_CONFIGS)
                                        .withCurrentLimits(currentLimitsConfigs));
        private final BayDoorMotor rightMotor = new BayDoorMotor(
                        "Right Bay Door Motor",
                        new CanId((byte) 15),
                        new TalonFXConfiguration()
                                        .withMotorOutput(new MotorOutputConfigs()
                                                        .withInverted(InvertedValue.Clockwise_Positive)
                                                        .withNeutralMode(NeutralModeValue.Brake))
                                        .withSlot0(SLOT0_CONFIGS)
                                        .withCurrentLimits(currentLimitsConfigs));

        private final BayDoorAbsoluteEncoder leftAbsoluteEncoder = new BayDoorAbsoluteEncoder(
                        "Left Bay Door Encoder",
                        new CanId((byte) 0),
                        new CANcoderConfiguration()
                                        .withMagnetSensor(
                                                        new MagnetSensorConfigs()
                                                                        .withMagnetOffset(Rotations.of(0))));
        private final BayDoorAbsoluteEncoder rightAbsoluteEncoder = new BayDoorAbsoluteEncoder(
                        "Right Bay Door Encoder",
                        new CanId((byte) 0),
                        new CANcoderConfiguration()
                                        .withMagnetSensor(
                                                        new MagnetSensorConfigs()
                                                                        .withMagnetOffset(Rotations.of(0))));

        private final Elastic.Notification homeCompletionNotification;
        private final Elastic.Notification homeIncompletionNotification;

        private static final Dimensionless HOME_DUTY_CYCLE = Percent.of(-15);
        private static final Angle OPEN_ANGLE_THRESHOLD = Rotations.of(6.3);
        private static final Angle OPEN_ANGLE_SETPOINT = Rotations.of(7.3);
        private static final Angle CLOSE_ANGLE_SETPOINT = Rotations.of(0);
        private static final Angle MIDDLE_ANGLE = OPEN_ANGLE_SETPOINT.div(2);
        private static final Angle MIDDLE_TOLERANCE = Degrees.of(2);
        private static final Dimensionless OPEN_PRESSURE_DUTY_CYCLE = Percent.of(5);
        private static final Dimensionless CLOSE_PRESSURE_DUTY_CYCLE = Percent.of(-5);

        public final Trigger atLeftCloseLimit;
        public final Trigger atRightCloseLimit;
        public final Trigger atLeftMiddleLimit;
        public final Trigger atRightMiddleLimit;
        public final Trigger atLeftOpenLimit;
        public final Trigger atRightOpenLimit;

        private final Trigger isBayDoorClosed;
        private final Trigger isBayDoorMiddle;
        private final Trigger isBayDoorOpen;

        public final Trigger hasBayDoorHomed = new Trigger(this::hasHomed);
        private boolean hasHomed = false;

        public BayDoor(String name) {
                super("Subsystems/" + name);
                homeCompletionNotification = new Notification(
                                Elastic.NotificationLevel.INFO,
                                name + " has been HOMED.",
                                "");
                homeIncompletionNotification = new Notification(
                                Elastic.NotificationLevel.WARNING,
                                name + " HOMING INCOMPLETE.",
                                "");

                atLeftCloseLimit = new Trigger(() -> leftAbsoluteEncoder.getAbsolutePosition().lte(CLOSE_ANGLE_SETPOINT));
                atLeftOpenLimit = new Trigger(() -> leftAbsoluteEncoder.getAbsolutePosition().gte(OPEN_ANGLE_THRESHOLD));
                
                atLeftMiddleLimit = new Trigger(() -> leftAbsoluteEncoder.getAbsolutePosition().isNear(MIDDLE_ANGLE, MIDDLE_TOLERANCE));
                atRightMiddleLimit = new Trigger(() -> rightAbsoluteEncoder.getAbsolutePosition().isNear(MIDDLE_ANGLE, MIDDLE_TOLERANCE));
                
                atRightCloseLimit = new Trigger(() -> rightAbsoluteEncoder.getAbsolutePosition().lte(CLOSE_ANGLE_SETPOINT));
                atRightOpenLimit = new Trigger(() -> rightAbsoluteEncoder.getAbsolutePosition().gte(OPEN_ANGLE_THRESHOLD));

                isBayDoorClosed = new Trigger(atLeftCloseLimit.and(atRightCloseLimit));
                isBayDoorMiddle = new Trigger(atLeftMiddleLimit.and(atRightMiddleLimit));
                isBayDoorOpen = new Trigger(atLeftOpenLimit.and(atRightCloseLimit));

                initSmartDashboard();
        }

        private void initSmartDashboard() {
                SmartDashboard.putData(getName(), this);
                SmartDashboard.putData(getName() + "/" + leftMotor.getSmartDashboardName(), leftMotor);
                SmartDashboard.putData(getName() + "/" + rightMotor.getSmartDashboardName(), rightMotor);
        }

        @Override
        public void initSendable(SendableBuilder builder) {
                super.initSendable(builder);

                builder.addBooleanProperty("atLeftCloseLimit", atLeftCloseLimit, null);
                builder.addBooleanProperty("atRightCloseLimit", atRightCloseLimit, null);

                builder.addBooleanProperty("atLeftSoftOpenLimit", atLeftOpenLimit, null);
                builder.addBooleanProperty("atRightSoftOpenLimit", atRightOpenLimit, null);

                builder.addBooleanProperty("isBayDoorClosed", isBayDoorClosed, null);
                builder.addBooleanProperty("isBayDoorMiddle", isBayDoorMiddle, null);
                builder.addBooleanProperty("isBayDoorOpen", isBayDoorOpen, null);

                builder.addBooleanProperty("hasBayDoorHomed", hasBayDoorHomed, null);
        }

        private boolean hasHomed() {
                return hasHomed;
        }

        private void setPointPosition(Angle angle) {
                leftMotor.setPointPosition(angle);
                rightMotor.setPointPosition(angle);
        }

        private void setDutyCycle(Dimensionless percent) {
                leftMotor.setDutyCycle(percent);
                rightMotor.setDutyCycle(percent);
        }

        public Command home() {
                return run(
                                () -> {
                                        leftMotor.home(atLeftCloseLimit.getAsBoolean(), HOME_DUTY_CYCLE);
                                        rightMotor.home(atRightCloseLimit.getAsBoolean(), HOME_DUTY_CYCLE);
                                })
                                .withName("Home")
                                .until(isBayDoorClosed)
                                .handleInterrupt(() -> hasHomed = false)
                                .finallyDo(interrupted -> {
                                        if (interrupted) {
                                                onHomingIncomplete();
                                        } else {
                                                onHomingComplete();
                                        }
                                })
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        }

        private Command hardHome() {
                return run(() -> {
                        leftMotor.setDutyCycle(HOME_DUTY_CYCLE);
                        rightMotor.setDutyCycle(HOME_DUTY_CYCLE);
                }).withDeadline(new WaitCommand(Seconds.of(0.5)))
                                .finallyDo(() -> {
                                        leftMotor.resetRelativeEncoder();
                                        rightMotor.resetRelativeEncoder();
                                        stop();
                                });
        }

        public Command ensuredHome() {
                return home().andThen(hardHome()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                                .finallyDo(interrupted -> {
                                        if (interrupted) {
                                                onHomingIncomplete();
                                        } else {
                                                onHomingComplete();
                                        }
                                });
        }

        private void onHomingComplete() {
                removeDefaultCommand();
                hasHomed = true;
                Elastic.sendNotification(homeCompletionNotification);
        }

        private void onHomingIncomplete() {
                setDefaultCommand(ensuredHome());
                hasHomed = false;
                Elastic.sendNotification(homeIncompletionNotification);
        }

        public Command open() {
                return runEnd(() -> setPointPosition(OPEN_ANGLE_SETPOINT), () -> setDutyCycle(OPEN_PRESSURE_DUTY_CYCLE))
                                .until(isBayDoorOpen)
                                .withName("Open");
        }

        public Command middle() {
                return run(() -> setPointPosition(MIDDLE_ANGLE))
                                .until(isBayDoorMiddle)
                                .withName("Middle");
        }

        public Command close() {
                return runEnd(() -> setPointPosition(CLOSE_ANGLE_SETPOINT),
                                () -> setDutyCycle(CLOSE_PRESSURE_DUTY_CYCLE))
                                .until(isBayDoorClosed)
                                .withName("Close");
        }

        private Command agitateFuel() {
                return new RepeatCommand(new ParallelRaceGroup(open(), new WaitCommand(Seconds.of(1)))
                                .andThen(new ParallelRaceGroup(close(), new WaitCommand(Seconds.of(0.3)))));
        }

        public Command agitateHighFuel() {
                return open().andThen(new WaitCommand(Seconds.of(2.5))).andThen(agitateFuel());
        }

        public Command agitateLowFuel() {
                return agitateFuel();
        }

        private void stop() {
                leftMotor.stop();
                rightMotor.stop();
        }
}

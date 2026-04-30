package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Watts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutPower;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hardware.CanId;
import frc.robot.hardware.motors.BayDoorMotor;
import frc.robot.hardware.sensors.BayDoorAbsoluteEncoder;

public class BayDoor extends SubsystemBase {
        private static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKG(0.04)
                        .withKS(0.05)
                        .withKP(2);

        private static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(25));

        private static final double MOTOR_SCALE_FACTOR = 25.0;

        private final BayDoorAbsoluteEncoder leftAbsoluteEncoder = new BayDoorAbsoluteEncoder(
                        "Left Bay Door Encoder",
                        new CanId((byte) 21),
                        new CANcoderConfiguration()
                                        .withMagnetSensor(
                                                        new MagnetSensorConfigs()
                                                                        .withSensorDirection(
                                                                                        SensorDirectionValue.CounterClockwise_Positive)
                                                                        .withAbsoluteSensorDiscontinuityPoint(
                                                                                        Rotations.of(1))
                                                                        .withMagnetOffset(
                                                                                        Rotations.of(-0.6572265625).plus(WEIRD_OFFSET_PREVENTION))));

        private final BayDoorAbsoluteEncoder rightAbsoluteEncoder = new BayDoorAbsoluteEncoder(
                        "Right Bay Door Encoder",
                        new CanId((byte) 22),
                        new CANcoderConfiguration()
                                        .withMagnetSensor(
                                                        new MagnetSensorConfigs()
                                                                        .withSensorDirection(
                                                                                        SensorDirectionValue.Clockwise_Positive)
                                                                        .withAbsoluteSensorDiscontinuityPoint(
                                                                                        Rotations.of(1))
                                                                        .withMagnetOffset(
                                                                                        Rotations.of(-0.2255859375).plus(WEIRD_OFFSET_PREVENTION))));

        private final BayDoorMotor leftMotor = new BayDoorMotor(
                        "Left Bay Door Motor",
                        new CanId((byte) 14),
                        new TalonFXConfiguration()
                                        .withMotorOutput(new MotorOutputConfigs()
                                                        .withInverted(InvertedValue.CounterClockwise_Positive)
                                                        .withNeutralMode(NeutralModeValue.Brake))
                                        .withSlot0(SLOT0_CONFIGS)
                                        .withCurrentLimits(currentLimitsConfigs)
                                        .withFeedback(new FeedbackConfigs()
                                                        .withRemoteCANcoder(leftAbsoluteEncoder.getEncoder())
                                                        .withRotorToSensorRatio(MOTOR_SCALE_FACTOR)));

        private final BayDoorMotor rightMotor = new BayDoorMotor(
                        "Right Bay Door Motor",
                        new CanId((byte) 15),
                        new TalonFXConfiguration()
                                        .withMotorOutput(new MotorOutputConfigs()
                                                        .withInverted(InvertedValue.Clockwise_Positive)
                                                        .withNeutralMode(NeutralModeValue.Brake))
                                        .withSlot0(SLOT0_CONFIGS)
                                        .withCurrentLimits(currentLimitsConfigs)
                                        .withFeedback(new FeedbackConfigs()
                                                        .withRemoteCANcoder(rightAbsoluteEncoder.getEncoder())
                                                        .withRotorToSensorRatio(MOTOR_SCALE_FACTOR)));

        private static final Angle WEIRD_OFFSET_PREVENTION = Rotations.of(0.1);
        private static final Angle OPEN_ANGLE_THRESHOLD = Rotations.of(0.225).plus(WEIRD_OFFSET_PREVENTION);
        private static final Angle OPEN_ANGLE_SETPOINT = OPEN_ANGLE_THRESHOLD;
        private static final Angle CLOSE_ANGLE_THRESHOLD = Degrees.of(5).plus(WEIRD_OFFSET_PREVENTION);
        private static final Angle CLOSE_ANGLE_SETPOINT = CLOSE_ANGLE_THRESHOLD;
        private static final Angle MIDDLE_ANGLE = OPEN_ANGLE_SETPOINT.div(1.5);
        private static final Angle MIDDLE_TOLERANCE = Degrees.of(2);
        private static final Dimensionless OPEN_PRESSURE_DUTY_CYCLE = Percent.of(3);
        private static final Dimensionless CLOSE_PRESSURE_DUTY_CYCLE = Percent.of(-3);

        public final Trigger atLeftCloseLimit;
        public final Trigger atRightCloseLimit;
        public final Trigger atLeftMiddleLimit;
        public final Trigger atRightMiddleLimit;
        public final Trigger atLeftOpenLimit;
        public final Trigger atRightOpenLimit;

        private final Trigger isBayDoorClosed;
        private final Trigger isBayDoorMiddle;
        private final Trigger isBayDoorOpen;

        public BayDoor(String name) {
                super("Subsystems/" + name);

                atLeftCloseLimit = new Trigger(
                                () -> leftAbsoluteEncoder.getAbsolutePosition().lte(CLOSE_ANGLE_THRESHOLD));
                atLeftOpenLimit = new Trigger(
                                () -> leftAbsoluteEncoder.getAbsolutePosition().gte(OPEN_ANGLE_THRESHOLD));

                atLeftMiddleLimit = new Trigger(
                                () -> leftAbsoluteEncoder.getAbsolutePosition().isNear(MIDDLE_ANGLE, MIDDLE_TOLERANCE));
                atRightMiddleLimit = new Trigger(() -> rightAbsoluteEncoder.getAbsolutePosition().isNear(MIDDLE_ANGLE,
                                MIDDLE_TOLERANCE));

                atRightCloseLimit = new Trigger(
                                () -> rightAbsoluteEncoder.getAbsolutePosition().lte(CLOSE_ANGLE_THRESHOLD));
                atRightOpenLimit = new Trigger(
                                () -> rightAbsoluteEncoder.getAbsolutePosition().gte(OPEN_ANGLE_THRESHOLD));

                isBayDoorClosed = new Trigger(atLeftCloseLimit.and(atRightCloseLimit));
                isBayDoorMiddle = new Trigger(atLeftMiddleLimit.and(atRightMiddleLimit));
                isBayDoorOpen = new Trigger(atLeftOpenLimit.and(atRightOpenLimit));
                initSmartDashboard();
        }

        private void initSmartDashboard() {
                SmartDashboard.putData(getName(), this);
                SmartDashboard.putData(getName() + "/" + leftMotor.getSmartDashboardName(), leftMotor);
                SmartDashboard.putData(getName() + "/" + rightMotor.getSmartDashboardName(), rightMotor);
                SmartDashboard.putData(getName() + "/" + leftAbsoluteEncoder.getSmartDashboardName(),
                                leftAbsoluteEncoder);
                SmartDashboard.putData(getName() + "/" + rightAbsoluteEncoder.getSmartDashboardName(),
                                rightAbsoluteEncoder);
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

                builder.addDoubleProperty("Power (Watts)", () -> getTotalPower().in(Watts), null);
        }

        private Power getTotalPower() {
                MutPower totalPower = Watts.zero().mutableCopy();
                totalPower.mut_plus(leftMotor.getPower());
                totalPower.mut_plus(rightMotor.getPower());
                return totalPower;
        }

        private void setPointPosition(Angle angle) {
                leftMotor.setPointPosition(angle);
                rightMotor.setPointPosition(angle);
        }

        private void setDutyCycle(Dimensionless percent) {
                leftMotor.setDutyCycle(percent);
                rightMotor.setDutyCycle(percent);
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

        public Command agitateFuel() {
                return open().raceWith(new WaitCommand(Seconds.of(0.5)))
                                .andThen(close().raceWith(new WaitCommand(Seconds.of(0.3)))).repeatedly()
                                .andThen(open());
        }

        public Command agitateFuel2() {
                return open().raceWith(new WaitCommand(Seconds.of(0.5)))
                                .andThen(middle().raceWith(new WaitCommand(Seconds.of(0.3)))).repeatedly()
                                .andThen(open());
        }

        public Command removeStuckFuel() {
                return open().andThen(close().raceWith(new WaitCommand(Seconds.of(0.3)))).repeatedly();
        }
}
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.Elastic;
import frc.robot.generated.Elastic.Notification;
import frc.robot.hardware.CanId;
import frc.robot.hardware.motors.ClimberMotor;

public class Climber extends SubsystemBase {
    private final ClimberMotor motor = new ClimberMotor(
            "Climber Motor",
            new CanId((byte) 0),
            new TalonFXConfiguration()
                    .withMotorOutput(new MotorOutputConfigs().withInverted(null))
                    // .withSlot0(new Slot0Configs()
                    // .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                    // .withGravityType(GravityTypeValue.Arm_Cosine)
                    // .withKG(null)
                    // .withKS(null)
                    // .withKP(null))
                    .withCurrentLimits(new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(null)
                            .withSupplyCurrentLimit(null))
                    .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                            .withForwardSoftLimitThreshold(null)
                            .withReverseSoftLimitThreshold(null)
                            .withForwardSoftLimitEnable(true)
                            .withReverseSoftLimitEnable(true)));

    private final Elastic.Notification homeNotification;

    private static final Angle OPEN_ANGLE_SETPOINT = Degrees.of(0); // TODO: What is this value?
    private static final Angle CLOSE_ANGLE_SETPOINT = Degrees.of(0);
    private static final Dimensionless HOME_DUTY_CYCLE = Percent.of(-5);

    public final Trigger atSoftOpen;
    public final Trigger atSoftClose;
    public final Trigger atHardClose;

    public Climber(String name) {
        super("Subsystems/" + name);
        homeNotification = new Notification(
                Elastic.NotificationLevel.INFO,
                name + " has been HOMED.",
                "");

        atSoftOpen = new Trigger(() -> motor.getAngle().gte(OPEN_ANGLE_SETPOINT));
        atSoftClose = new Trigger(() -> motor.getAngle().lte(CLOSE_ANGLE_SETPOINT));
        atHardClose = new Trigger(() -> false); // TODO: What is going to use verify that this will home.
        initSmartDashboard();
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(motor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    public Command open() {
        return run(() -> motor.setPointPosition(OPEN_ANGLE_SETPOINT));
    }

    public Command close() {
        return runEnd(() -> motor.setPointPosition(CLOSE_ANGLE_SETPOINT), () -> motor.setDutyCycle(Percent.of(-3)));
    }

    public Command home() {
        return runEnd(
                () -> motor.home(atHardClose.getAsBoolean(), HOME_DUTY_CYCLE),
                () -> {
                    removeDefaultCommand();
                    Elastic.sendNotification(homeNotification);
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
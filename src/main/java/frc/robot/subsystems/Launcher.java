package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.Elastic;
import frc.robot.generated.Elastic.Notification;
import frc.robot.generated.Elastic.NotificationLevel;
import frc.robot.hardware.CanId;
import frc.robot.hardware.motors.LauncherMotor;

public class Launcher extends SubsystemBase {
    private static final Slot0Configs slot0Configs = new Slot0Configs()
            .withKP(0.04)
            .withKS(0.0185)
            .withKV(0.0098);

    private static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(2500));

    private static final NeutralModeValue neutralModeValue = NeutralModeValue.Coast;
    
    private static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(80));

    private final LauncherMotor leadMotor = new LauncherMotor(
            "Left Motor",
            new CanId((byte) 18),
            new TalonFXConfiguration()
                    .withMotorOutput(new MotorOutputConfigs()
                            .withInverted(InvertedValue.Clockwise_Positive)
                            .withNeutralMode(neutralModeValue))
                    .withCurrentLimits(currentLimitsConfigs)
                    .withMotionMagic(motionMagicConfigs)
                    .withSlot0(slot0Configs));
    private final LauncherMotor followerMotor = new LauncherMotor(
            "Right Motor",
            new CanId((byte) 19),
            new TalonFXConfiguration()
                    .withMotorOutput(new MotorOutputConfigs()
                            .withInverted(InvertedValue.CounterClockwise_Positive)
                            .withNeutralMode(neutralModeValue))
                    .withCurrentLimits(currentLimitsConfigs)
                    .withMotionMagic(motionMagicConfigs)
                    .withSlot0(slot0Configs));

    private static final AngularVelocity PREP_ANGULAR_VELOCITY = RPM.of(1500);
    private static final AngularVelocity NEAR_TARGET_VELOCITY_THRESHHOLD = RPM.of(100);
    private static final AngularVelocity MAX_USER_VELOCITY_OFFSET = RPM.of(200);
    private static final AngularVelocity MIN_USER_VELOCITY_OFFSET = RPM.of(-200);

    private AngularVelocity smartDashboardLaunchVelocity = RotationsPerSecond.of(0);
    private AngularVelocity launcherOffset = RPM.of(0);

    public final Trigger nearTargetRPM;

    private final static String incrementLauncherOffsetTitle = "Launcher Offset increments to: ";
    private final static String decrementLauncherOffsetTitle = "Launcher Offset decrements to: ";
    private final Elastic.Notification launcherOffsetNotification = new Notification(NotificationLevel.INFO, "", "")
            .withDisplaySeconds(0.8);

    public Launcher(String name) {
        super("Subsystems/" + name);

        followerMotor.follow(leadMotor.getCanId(), MotorAlignmentValue.Opposed);

        addChild(getName(), leadMotor);
        addChild(getName(), followerMotor);

        nearTargetRPM = new Trigger(
                () -> leadMotor.getVelocity().isNear(leadMotor.getTargetVelocity(), NEAR_TARGET_VELOCITY_THRESHHOLD));

        launcherOffsetNotification.setWidth(250);
        launcherOffsetNotification.setHeight(75);
        initSmartDashboard();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(
                "Motor Shoot Velocity (RPM)",
                () -> getSmartDashboardLaunchTargetVelocity().in(RPM),
                velocity -> setSmartDashboardLaunchTargetVelocity(RPM.of(velocity)));

        builder.addDoubleProperty(
                "Launcher Offset (RPM)",
                () -> getRPMLauncherOffset().in(RPM),
                this::setRPMLauncherOffset);

        builder.addBooleanProperty("Near Target Velocity (RPM)", nearTargetRPM, null);
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + leadMotor.getSmartDashboardName(), leadMotor);
        SmartDashboard.putData(getName() + "/" + followerMotor.getSmartDashboardName(), followerMotor);
    }

    private void setRPMLauncherOffset(double rpm) {
        launcherOffset = RPM.of(
                MathUtil.clamp(
                        rpm,
                        MIN_USER_VELOCITY_OFFSET.in(RPM),
                        MAX_USER_VELOCITY_OFFSET.in(RPM)));
    }

    public Command incrementLauncherOffset() {
        return new InstantCommand(() -> {
            setRPMLauncherOffset(getRPMLauncherOffset().plus(RPM.of(10)).in(RPM));
            Elastic.sendNotification(launcherOffsetNotification
                    .withTitle(incrementLauncherOffsetTitle + getRPMLauncherOffset().in(RPM)));
        });
    }

    public Command decrementLauncherOffset() {
        return new InstantCommand(() -> {
            setRPMLauncherOffset(getRPMLauncherOffset().minus(RPM.of(10)).in(RPM));
            Elastic.sendNotification(launcherOffsetNotification
                    .withTitle(decrementLauncherOffsetTitle + getRPMLauncherOffset().in(RPM)));
        });
    }

    private AngularVelocity getRPMLauncherOffset() {
        return launcherOffset;
    }

    public Command launchFuel(Supplier<AngularVelocity> velocity) {
        return runEnd(() -> leadMotor.setPointVelocity(velocity.get().plus(launcherOffset)), this::stop);
    }

    public Command prepareLaunch() {
        return runEnd(() -> {
            leadMotor.setPointVelocity(PREP_ANGULAR_VELOCITY);
        }, this::stop);
    }

    public Command smartDashboardLaunchFuel() {
        return runEnd(() -> leadMotor.setPointVelocity(getSmartDashboardLaunchTargetVelocity()), this::stop);
    }

    public AngularVelocity getSmartDashboardLaunchTargetVelocity() {
        return smartDashboardLaunchVelocity;
    }

    public AngularVelocity getLaunchVelocity() {
        return leadMotor.getVelocity();
    }

    public void setSmartDashboardLaunchTargetVelocity(AngularVelocity velocity) {
        smartDashboardLaunchVelocity = velocity;
    }

    private void stop() {
        leadMotor.stop();
    }
}

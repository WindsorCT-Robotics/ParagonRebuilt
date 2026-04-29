package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Watts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutPower;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hardware.CanId;
import frc.robot.hardware.motors.LauncherMotor;

public class Launcher extends SubsystemBase {
    private static final Slot0Configs slot0Configs = new Slot0Configs()
            .withKP(0.025)
            .withKS(0.0185)
            .withKV(0.0098);

    private static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(150));

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
    private static final AngularVelocity NEAR_TARGET_VELOCITY_TOLERANCE = RPM.of(2000);

    private AngularVelocity smartDashboardLaunchVelocity = RPM.of(2000);
    private AngularVelocity launcherOffset = RPM.of(0);

    public final Trigger nearTargetVelocity;

    public Launcher(String name) {
        super("Subsystems/" + name);

        followerMotor.follow(leadMotor.getCanId(), MotorAlignmentValue.Opposed);

        addChild(getName(), leadMotor);
        addChild(getName(), followerMotor);

        nearTargetVelocity = new Trigger(this::isNearTargetVelocity);

        initSmartDashboard();
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + leadMotor.getSmartDashboardName(), leadMotor);
        SmartDashboard.putData(getName() + "/" + followerMotor.getSmartDashboardName(), followerMotor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(
                "Motor Shoot Velocity (RPM)",
                () -> getSmartDashboardLaunchTargetVelocity().in(RPM),
                velocity -> setSmartDashboardLaunchTargetVelocity(RPM.of(velocity)));

        builder.addBooleanProperty("Near Target Velocity (RPM)", nearTargetVelocity, null);

        builder.addDoubleProperty("Power (Watts)", () -> getTotalPower().in(Watts), null);
    }

    private Power getTotalPower() {
        MutPower totalPower = Watts.zero().mutableCopy();
        totalPower.mut_plus(leadMotor.getPower());
        totalPower.mut_plus(followerMotor.getPower());
        return totalPower;
    }

    public Command launchFuel(Supplier<AngularVelocity> velocity) {
        return runEnd(() -> leadMotor.setPointVelocity(velocity.get().plus(launcherOffset)), this::stop);
    }

    public Command prepareFuel() {
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

    private boolean isNearTargetVelocity() {
        AngularVelocity velocity = leadMotor.getVelocity().plus(followerMotor.getVelocity()).div(2);
        AngularVelocity targetThreshold = leadMotor.getTargetVelocity()
                .plus(followerMotor.getTargetVelocity())
                .div(2)
                .minus(NEAR_TARGET_VELOCITY_TOLERANCE);

        return velocity.gte(targetThreshold);
    }
}

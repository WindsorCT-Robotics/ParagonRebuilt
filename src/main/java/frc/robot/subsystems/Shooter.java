package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Value;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.interfaces.ISystemDynamics;
import frc.robot.hardware.CanId;
import frc.robot.hardware.motors.ShooterMotor;

public class Shooter extends SubsystemBase implements ISystemDynamics<ShooterMotor> {
    private final ShooterMotor leadMotor;
    private final ShooterMotor followerMotor;
    private final SysIdRoutine routine;
    private static final Distance HALF_FIELD = Meters
            .of(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark).getFieldLength() / 3);
    private static final AngularVelocity PREP_ANGULAR_VELOCITY = RPM.of(1500);
    private AngularVelocity smartDashboardLaunchVelocity = RotationsPerSecond.of(0);
    private AngularVelocity launcherOffset = RPM.of(0);

    public Shooter(String name, CanId leftMotorId, CanId rightMotorId) {
        super("Subsystems/" + name);
        final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(2500));
        final Slot0Configs slot0Configs = new Slot0Configs()
                .withKS(0.03)
                .withKV(0.0105);
        final NeutralModeValue neutralModeValue = NeutralModeValue.Coast;

        leadMotor = new ShooterMotor("Left Motor", leftMotorId, new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(neutralModeValue))
                .withMotionMagic(motionMagicConfigs)
                .withSlot0(slot0Configs));
        followerMotor = new ShooterMotor("Right Motor", rightMotorId, new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(neutralModeValue))
                .withMotionMagic(motionMagicConfigs)
                .withSlot0(slot0Configs));

        followerMotor.follow(leftMotorId.Id(), MotorAlignmentValue.Opposed);

        addChild(getName(), leadMotor);
        addChild(getName(), followerMotor);

        routine = new SysIdRoutine(new Config(), new Mechanism(this::setSysIdVoltage, log -> {
            log(log, leadMotor, "Left Shooter Motor");
            log(log, followerMotor, "Right Shooter Motor");
        }, this));

        initSmartDashboard();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Motor Shoot Velocity (RPM)",
                () -> getSmartDashboardLaunchTargetVelocity().in(RPM),
                velocity -> setSmartDashboardLaunchTargetVelocity(RPM.of(velocity)));
        builder.addDoubleProperty("Launcher Offset (RPM)", () -> getRPMLauncherOffset().in(RPM),
                this::setRPMLauncherOffset);
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + leadMotor.getSmartDashboardName(), leadMotor);
        SmartDashboard.putData(getName() + "/" + followerMotor.getSmartDashboardName(), followerMotor);
    }

    private void setRPMLauncherOffset(double rpm) {
        launcherOffset = RPM.of(rpm);
    }

    private AngularVelocity getRPMLauncherOffset() {
        return launcherOffset;
    }

    public Command launchFuel(Supplier<AngularVelocity> velocity) {
        return runEnd(() -> leadMotor.setPointVelocity(velocity.get()), this::stop);
    }

    public Command launchFuelAdjustableToHub(
            Supplier<AngularVelocity> velocity,
            Supplier<Dimensionless> adjustment,
            AngularVelocity maxAdjustment,
            Trigger onAllianceSide) {
        return runEnd(
                () -> {
                    if (onAllianceSide.getAsBoolean()) {
                        leadMotor.setPointVelocity(
                                velocity.get()
                                        .plus(maxAdjustment.times(adjustment.get().in(Value)))
                                        .plus(launcherOffset));
                    } else {
                        leadMotor.setPointVelocity(RPM.zero());
                    }
                },
                this::stop);
    }

    public Command smartDashboardLaunchFuel() {
        return runEnd(() -> leadMotor.setPointVelocity(getSmartDashboardLaunchTargetVelocity()), this::stop);
    }

    public Command prepareLaunch(Supplier<Pose2d> robotPosition) {
        return runEnd(() -> {
            Optional<Alliance> maybeAlliance = DriverStation.getAlliance();

            AngularVelocity velocity = RPM.zero();

            velocity = maybeAlliance.map(alliance -> {
                if (alliance == Alliance.Blue && robotPosition.get().getMeasureX().lt(HALF_FIELD))
                    return PREP_ANGULAR_VELOCITY;

                if (alliance == Alliance.Red && robotPosition.get().getMeasureX().gt(HALF_FIELD))
                    return PREP_ANGULAR_VELOCITY;

                return RPM.zero();
            }).orElse(RPM.zero());

            leadMotor.setPointVelocity(velocity);
        }, this::stop);
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

    // region SysId
    private void setSysIdVoltage(Voltage voltage) {
        leadMotor.setVoltage(voltage);
    }

    @Override
    public void log(SysIdRoutineLog log, ShooterMotor motor, String name) {
        log.motor(name).angularPosition(motor.getAngle()).angularVelocity(motor.getVelocity());
    }

    @Override
    public Command sysIdDynamic(Direction direction) {
        return routine.dynamic(direction).withName(getSubsystem() + "/sysIdDynamic");
    }

    @Override
    public Command sysIdQuasistatic(Direction direction) {
        return routine.quasistatic(direction).withName(getSubsystem() + "/sysIdQuasistatic");
    }
    // endregion
}

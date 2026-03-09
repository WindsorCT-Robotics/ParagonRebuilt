package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private AngularVelocity shootVelocity = RotationsPerSecond.of(0);

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

    public Command shootFuel(Supplier<AngularVelocity> shootVelocity) {
        return runEnd(() -> leadMotor.setPointVelocity(shootVelocity.get()), this::stop);
    }

    public Command shootFuelSmartDashboard() {
        return runEnd(() -> leadMotor.setPointVelocity(getShootTargetVelocity()), this::stop);
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + leadMotor.getSmartDashboardName(), leadMotor);
        SmartDashboard.putData(getName() + "/" + followerMotor.getSmartDashboardName(), followerMotor);
    }

    public AngularVelocity getShootTargetVelocity() {
        return shootVelocity;
    }

    public AngularVelocity getShootVelocity() {
        return leadMotor.getVelocity();
    }

    public void setShootTargetVelocity(double rpm) {
        shootVelocity = RPM.of(rpm);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Motor Shoot Velocity (RPM)",
                () -> getShootTargetVelocity().in(RPM),
                this::setShootTargetVelocity);
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

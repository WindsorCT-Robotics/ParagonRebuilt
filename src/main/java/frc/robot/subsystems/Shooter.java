package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.hardware.basic_implementations.shooter_motors.ShooterMotorBasic;
import frc.robot.interfaces.ISystemDynamics;
import frc.robot.hardware.CanId;

public class Shooter extends SubsystemBase implements ISystemDynamics<ShooterMotorBasic> {
    private final ShooterMotorBasic leadMotor;
    private final ShooterMotorBasic followerMotor;

    private final SysIdRoutine routine;
    private static final boolean INVERTED = true;

    private AngularVelocity shootVelocity = RotationsPerSecond.of(0);
    private AngularVelocity retreatVelocity = RotationsPerSecond.of(0);

    private static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(0)
            .withMotionMagicExpo_kV(null)
            .withMotionMagicExpo_kA(null);
    private static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKV(0)
            .withKA(0)
            .withKG(0)
            .withGravityType(GravityTypeValue.Elevator_Static);

    public Shooter(String name, CanId leftMotorId, CanId rightMotorId) {
        super("Subsystems/" + name);
        leadMotor = new ShooterMotorBasic("Left Motor", leftMotorId, this::setDutyCycle, this::setVoltage);
        followerMotor = new ShooterMotorBasic("Right Motor", rightMotorId, this::setDutyCycle,
                this::setVoltage);

        leadMotor.configure(motor -> {
            TalonFXConfigurator configurator = motor.getConfigurator();
            configurator.apply(new MotorOutputConfigs().withInverted(
                    (INVERTED) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive));
            configurator.apply(MOTION_MAGIC_CONFIGS);
            configurator.apply(SLOT0_CONFIGS);
        });

        followerMotor.follow(leftMotorId.Id(), MotorAlignmentValue.Opposed);

        addChild(getName(), leadMotor);
        addChild(getName(), followerMotor);

        routine = new SysIdRoutine(new Config(), new Mechanism(this::setSysIdVoltage, log -> {
            log(log, leadMotor, "Left Shooter Motor");
            log(log, followerMotor, "Right Shooter Motor");
        }, this));

        initSmartDashboard();
    }

    public Command shootFuel() {
        return runEnd(() -> leadMotor.setPointVelocity(getShootTargetVelocity()), () -> stop());
    }

    public Command retreatFuel() {
        return runEnd(() -> leadMotor.setPointVelocity(getRetreatTargetVelocity()), () -> stop());
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + leadMotor.getSmartDashboardName(), leadMotor);
        SmartDashboard.putData(getName() + "/" + followerMotor.getSmartDashboardName(), followerMotor);
    }

    private AngularVelocity getShootTargetVelocity() {
        return shootVelocity;
    }

    private void setShootTargetVelocity(double RPS) {
        shootVelocity = RotationsPerSecond.of(RPS);
    }

    private AngularVelocity getRetreatTargetVelocity() {
        return retreatVelocity;
    }

    private void setRetreatTargetVelocity(double RPS) {
        retreatVelocity = RotationsPerSecond.of(RPS);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Motor Shoot Velocity (RPS)",
                () -> getShootTargetVelocity().in(RotationsPerSecond),
                this::setShootTargetVelocity);
        builder.addDoubleProperty("Motor Retreat Velocity (RPS)",
                () -> getRetreatTargetVelocity().in(RotationsPerSecond),
                this::setRetreatTargetVelocity);
    }

    private void stop() {
        leadMotor.stop();
    }

    // region SysId
    private void setSysIdVoltage(Voltage voltage) {
        leadMotor.setVoltage(voltage);
    }

    private void setVoltage(Voltage voltage) {
        CommandScheduler.getInstance().schedule(overrideMotorVoltage(voltage));
    }

    public Command overrideMotorVoltage(Voltage voltage) {
        return runEnd(() -> {
            leadMotor.setVoltage(voltage);
        }, this::stop);
    }

    private void setDutyCycle(Dimensionless dutyCycle) {
        CommandScheduler.getInstance().schedule(overrideMotorDutyCycle(dutyCycle));
    }

    public Command overrideMotorDutyCycle(Dimensionless dutyCycle) {
        return runEnd(() -> {
            leadMotor.setDutyCycle(dutyCycle);
        }, this::stop).withName(getSubsystem() + "/overrideMotorDutyCycle");
    }

    @Override
    public void log(SysIdRoutineLog log, ShooterMotorBasic motor, String name) {
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

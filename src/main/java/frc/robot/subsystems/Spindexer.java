package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.hardware.CanId;
import frc.robot.hardware.spindexer_motor.SpindexterMotor;
import frc.robot.interfaces.ISystemDynamics;

public class Spindexer extends SubsystemBase implements ISystemDynamics<SpindexterMotor> {
    private final SpindexterMotor motor;

    private final SysIdRoutine routine;

    // TODO: Determine RPS.
    private static final AngularVelocity INDEX_VELOCITY = RotationsPerSecond.of(0);
    private static final AngularVelocity SHUTTLE_VELOCITY = RotationsPerSecond.of(0);

    private static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(KICK_FUEL_VELOCITY)
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

    public Spindexer(
            String name,
            CanId motorCanId) {
        super("Subsystems/" + name);
        motor = new SpindexterMotor("Motor", motorCanId, this::setDutyCycle, this::setVoltage);
        addChild(this.getName(), motor);

        routine = new SysIdRoutine(new Config(),
                new Mechanism(this::setSysIdVoltage, log -> log(log, motor, "Spindexer Motor"), this));
        initSmartDashboard();
    }

    // TODO: Make this a target Rotation Per Second instead of a duty cycle
    public Command indexFuel() {
        return runOnce(() -> motor.setPointVelocity(INDEX_VELOCITY));
    }

    // TODO: Make this a target Rotation Per Second instead of a duty cycle
    public Command releaseFuel() {
        return runOnce(() -> motor.setPointVelocity(SHUTTLE_VELOCITY));
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + motor.getClass().getSimpleName(), motor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    // region SysId
    private void setVoltage(Voltage voltage) {
        CommandScheduler.getInstance().schedule(overrideMotorVoltage(voltage));
    }

    public Command overrideMotorVoltage(Voltage voltage) {
        return runEnd(() -> {
            motor.setVoltage(voltage);
        }, () -> motor.stop());
    }

    private void setDutyCycle(Dimensionless dutyCycle) {
        CommandScheduler.getInstance().schedule(overrideMotorDutyCycle(dutyCycle));
    }

    public Command overrideMotorDutyCycle(Dimensionless dutyCycle) {
        return runEnd(() -> {
            motor.setDutyCycle(dutyCycle);
        }, () -> motor.stop()).withName(getSubsystem() + "/overrideMotorDutyCycle");
    }

    private void setSysIdVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void log(SysIdRoutineLog log, SpindexterMotor motor, String name) {
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
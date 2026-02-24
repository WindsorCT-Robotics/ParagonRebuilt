package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private static final Dimensionless INDEX_DUTY_CYCLE = Percent.of(100);

    public Spindexer(
            String name,
            CanId motorCanId) {
        super("Subsystems/" + name);
        motor = new SpindexterMotor(name, motorCanId);
        addChild(this.getName(), motor);
        routine = new SysIdRoutine(new Config(),
                new Mechanism(this::setVoltage, log -> log(log, motor, "Spindexer Motor"), this));
        initSmartDashboard();
    }

    private void setVoltage(Voltage voltage) {
        CommandScheduler.getInstance().schedule(overrideMotorVoltage(voltage));
    }

    @Override
    public void log(SysIdRoutineLog log, SpindexterMotor motor, String name) {
        log.motor(name).angularPosition(motor.getAngle()).angularVelocity(motor.getVelocity());
    }

    public Command overrideMotorVoltage(Voltage voltage) {
        return run(() -> motor.setVoltage(voltage));
    }

    @Override
    public Command sysIdDynamic(Direction direction) {
        return routine.dynamic(direction);
    }

    @Override
    public Command sysIdQuasistatic(Direction direction) {
        return routine.quasistatic(direction);
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + motor.getClass().getSimpleName(), motor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    // TODO: Make this a target Rotation Per Second instead of a duty cycle
    public Command indexFuel() {
        return Commands.runEnd(() -> motor.setDutyCycle(INDEX_DUTY_CYCLE), () -> motor.stop());
    }

    // TODO: Make this a target Rotation Per Second instead of a duty cycle
    public Command releaseFuel() {
        return Commands.runEnd(() -> motor.setDutyCycle(INDEX_DUTY_CYCLE.times(-1)), () -> motor.stop());
    }
}
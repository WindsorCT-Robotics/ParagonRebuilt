package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.hardware.shooter_motors.KickerMotor;
import frc.robot.interfaces.ISystemDynamics;
import frc.robot.hardware.CanId;

public class Kicker extends SubsystemBase implements ISystemDynamics<KickerMotor> {
    private final KickerMotor motor;
    private final SysIdRoutine routine;
    private static final Dimensionless DEFAULT_DUTY_CYCLE = Percent.of(10);

    public Kicker(String name, CanId motorId) {
        super("Subsystems/" + name);
        motor = new KickerMotor("Kicker Motor", motorId);
        motor.configure(motor -> {
            motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        });
        addChild(motor.getClass().getName(), motor);
        routine = new SysIdRoutine(new Config(),
                new Mechanism(this::setVoltage, log -> log(log, motor, "Kicker Motor"), this));
        initSmartDashboard();
    }

    private void setVoltage(Voltage voltage) {
        CommandScheduler.getInstance().schedule(overrideMotorVoltage(voltage));
    }

    @Override
    public void log(SysIdRoutineLog log, KickerMotor motor, String name) {
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
    public Command kickStartFuel() {
        return Commands.runEnd(() -> motor.setDutyCycle(DEFAULT_DUTY_CYCLE), () -> motor.stop());
    }
}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.hardware.CanId;
import frc.robot.hardware.intake_motors.IntakeRollerMotor;
import frc.robot.interfaces.ISystemDynamics;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Intake extends SubsystemBase implements ISystemDynamics<IntakeRollerMotor> {
    private final IntakeRollerMotor motor;
    private static final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0, 0, TimedRobot.kDefaultPeriod); // TODO: Configure with
                                                                                             // SysId Routines.
    private final SysIdRoutine routine;

    private static final Dimensionless INTAKE_FUEL_DUTY_CYCLE = Percent.of(20);
    private static final Dimensionless SHUTTLE_FUEL_DUTY_CYCLE = Percent.of(-20);

    public Intake(String name, CanId motorCanId) {
        super("Subsystems/" + name);
        motor = new IntakeRollerMotor(name, motorCanId, ff, this::setDutyCycle, this::setVelocity, this::setVoltage);
        addChild(motor.getClass().getName(), motor);
        // TODO: Consider customizing new Config(). Should be customized if motor has
        // physical limitations.
        routine = new SysIdRoutine(
                new Config(),
                new Mechanism(this::setSysIdVoltage,
                        log -> log(log, motor, "IntakeRollerMotor"), this));
        initSmartDashboard();
    }

    // TODO: Make this a target Rotation Per Second instead of a duty cycle
    public Command intakeFuel() {
        return runEnd(() -> motor.setDutyCycle(INTAKE_FUEL_DUTY_CYCLE), () -> motor.stop())
                .withName(getSubsystem() + "/intakeFuel");
    }

    // TODO: Make this a target Rotation Per Second instead of a duty cycle
    public Command shuttleFuel() {
        return runEnd(() -> motor.setDutyCycle(SHUTTLE_FUEL_DUTY_CYCLE), () -> motor.stop())
                .withName(getSubsystem() + "/shuttleFuel");
    }

    public Command stopIntake() {
        return run(() -> motor.stop()).withName(getSubsystem() + "/stopIntake");
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + motor.getClass().getSimpleName(), motor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    private void setDutyCycle(Dimensionless dutyCycle) {
        CommandScheduler.getInstance().schedule(overrideMotorDutyCycle(dutyCycle));
    }

    private void setVelocity(AngularVelocity velocity) {
        CommandScheduler.getInstance().schedule(overrideMotorVelocity(velocity));
    }

    private void setVoltage(Voltage v) {
        CommandScheduler.getInstance().schedule(overrideMotorVoltage(v));
    }

    private void setSysIdVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    public Command overrideMotorDutyCycle(Dimensionless dutyCycle) {
        return runEnd(() -> {
            motor.setDutyCycle(dutyCycle);
        }, () -> motor.stop()).withName(getSubsystem() + "/overrideMotorDutyCycle");
    }

    public Command overrideMotorVelocity(AngularVelocity velocity) {
        return runEnd(() -> {
            motor.setVelocity(velocity);
        }, () -> motor.stop()).withName(getSubsystem() + "/overrideMotorVelocity");
    }

    public Command overrideMotorVoltage(Voltage v) {
        return runEnd(() -> {
            motor.setVoltage(v);
        }, () -> motor.stop()).withName(getSubsystem() + "/overrideMotorVoltage");
    }

    @Override
    public void log(SysIdRoutineLog log, IntakeRollerMotor motor, String name) {
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
}
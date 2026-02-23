package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.hardware.CanId;
import frc.robot.hardware.intake_motors.IntakeRollerMotor;
import frc.robot.interfaces.ISystemDynamics;

public class Intake extends SubsystemBase implements ISystemDynamics {
    private final IntakeRollerMotor motor;

    private static final Dimensionless INTAKE_FUEL_DUTY_CYCLE = Percent.of(20);
    private static final Dimensionless SHUTTLE_FUEL_DUTY_CYCLE = Percent.of(-20);

    public Intake(String name, CanId motorCanId) {
        super("Subsystems/" + name);
        motor = new IntakeRollerMotor(name, motorCanId, this::setDutyCycle, this::setVelocity, this::setVoltage);
        addChild(motor.getClass().getName(), motor);
        initSmartDashboard();
    }

    @Override
    public Command sysIdDynamic(Direction direction) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Command sysIdQuasistatic(Direction direction) {
        // TODO Auto-generated method stub
        return null;
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

    private void setDutyCycle(Dimensionless dutyCycle) {
        CommandScheduler.getInstance().schedule(overrideMotorDutyCycle(dutyCycle));
    }

    private void setVelocity(AngularVelocity velocity) {
        CommandScheduler.getInstance().schedule(overrideMotorVelocity(velocity));
    }

    private void setVoltage(Voltage v) {
        CommandScheduler.getInstance().schedule(overrideMotorVoltage(v));
    }

    public Command overrideMotorDutyCycle(Dimensionless dutyCycle) {
        return run(() -> {
            motor.setDutyCycle(dutyCycle);
        }).withName(getSubsystem() + "/overrideMotorDutyCycle");
    }

    public Command overrideMotorVelocity(AngularVelocity velocity) {
        return run(() -> {
            motor.setVelocity(velocity);
        }).withName(getSubsystem() + "/overrideMotorVelocity");
    }

    public Command overrideMotorVoltage(Voltage v) {
        return run(() -> {
            motor.setVoltage(v);
        }).withName(getSubsystem() + "/overrideMotorVoltage");
    }
}
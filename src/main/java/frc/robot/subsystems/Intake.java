package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.CanId;
import frc.robot.hardware.intake_motors.IntakeRollerMotor;

public class Intake extends SubsystemBase {
    private final IntakeRollerMotor motor;

    private static final Dimensionless INTAKE_FUEL_DUTY_CYCLE = Percent.of(20);
    private static final Dimensionless SHUTTLE_FUEL_DUTY_CYCLE = Percent.of(-20);

    public Intake(String name, CanId motorCanId) {
        super(name);
        motor = new IntakeRollerMotor(name, motorCanId, this::setDutyCycle, this::setVelocity, this::setVoltage);
    }

    // TODO: Make this a target Rotation Per Second instead of a duty cycle
    public Command intakeFuel() {
        return runEnd(() -> motor.setDutyCycle(INTAKE_FUEL_DUTY_CYCLE), () -> motor.stop());
    }

    // TODO: Make this a target Rotation Per Second instead of a duty cycle
    public Command shuttleFuel() {
        return runEnd(() -> motor.setDutyCycle(SHUTTLE_FUEL_DUTY_CYCLE), () -> motor.stop());
    }

    public Command stopIntake() {
        return run(() -> motor.stop());
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
        });
    }

    public Command overrideMotorVelocity(AngularVelocity velocity) {
        return run(() -> {
            motor.setVelocity(velocity);
        });
    }

    public Command overrideMotorVoltage(Voltage v) {
        return run(() -> {
            motor.setVoltage(v);
        });
    }
}
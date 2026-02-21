package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.CanId;
import frc.robot.hardware.intake_motors.IntakeRollerMotor;

public class Intake extends SubsystemBase {
    private final IntakeRollerMotor motor;

    private static final Dimensionless INTAKE_FUEL_DUTY_CYCLE = Percent.of(100);
    private static final Dimensionless SHUTTLE_FUEL_DUTY_CYCLE = Percent.of(-100);

    public Intake(String name, CanId motorCanId) {
        SendableRegistry.add(this, name);
        motor = new IntakeRollerMotor(name, motorCanId);
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
}
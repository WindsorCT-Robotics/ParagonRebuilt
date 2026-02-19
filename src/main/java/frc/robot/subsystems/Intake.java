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

    private static final Dimensionless INTAKE_FUEL_DUTY_CYCLE = Percent.of(20);
    private static final Dimensionless SHUTTLE_FUEL_DUTY_CYCLE = Percent.of(-20);

    public Intake(String name, CanId motorCanId) {
        SendableRegistry.add(this, name);
        motor = new IntakeRollerMotor(name, motorCanId);
    }

    public Command intakeFuel() {
        return runEnd(() -> motor.setDutyCycle(INTAKE_FUEL_DUTY_CYCLE), () -> motor.stop());
    }

    public Command shuttleFuel() {
        return runEnd(() -> motor.setDutyCycle(SHUTTLE_FUEL_DUTY_CYCLE), () -> motor.stop());
    }
}
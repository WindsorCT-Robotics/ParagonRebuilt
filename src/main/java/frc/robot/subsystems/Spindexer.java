package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.CanId;
import frc.robot.hardware.spindexer_motor.SpindexterMotor;

public class Spindexer extends SubsystemBase {
    private final SpindexterMotor motor;

    private static final Dimensionless INDEX_DUTY_CYCLE = Percent.of(20);

    public Spindexer(
            String name,
            CanId motorCanId) {
        SendableRegistry.add(this, name);
        motor = new SpindexterMotor(name, motorCanId);
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }

    public Command indexFuel() {
        return Commands.runEnd(() -> motor.setDutyCycle(INDEX_DUTY_CYCLE), () -> motor.stop());
    }

    public Command releaseFuel() {
        return Commands.runEnd(() -> motor.setDutyCycle(INDEX_DUTY_CYCLE.times(-1)), () -> motor.stop());
    }
}
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.shooter_motors.KickerMotor;
import frc.robot.hardware.CanId;


public class Kicker extends SubsystemBase {
    private final KickerMotor motor;
    private static final Dimensionless DEFAULT_DUTY_CYCLE = Percent.of(10);

    public Kicker(String name, CanId motorId) {
        motor = new KickerMotor("Kicker Motor", motorId);
        motor.setInverted(InvertedValue.Clockwise_Positive);
    }

    public Command kickStartFuel() {
        return Commands.runEnd(() -> motor.setDutyCycle(DEFAULT_DUTY_CYCLE), () -> motor.stop());
    }
}

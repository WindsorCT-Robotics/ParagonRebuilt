package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.basic_implementations.shooter_motors.ShooterMotorBasic;
import frc.robot.hardware.CanId;

public class Shooter extends SubsystemBase {
    private final ShooterMotorBasic leftMotor;
    private final ShooterMotorBasic rightMotor;
    private static final boolean INVERTED = false;
    private static final Dimensionless DEFAULT_DUTY_CYCLE = Percent.of(10);


    public Shooter(String name, CanId leftMotorId, CanId rightMotorId) {
        leftMotor = new ShooterMotorBasic("Left Shooter Motor", leftMotorId);
        rightMotor = new ShooterMotorBasic("Right Shooter Motor", rightMotorId);

        leftMotor.setInverted((INVERTED) ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive);
        rightMotor.setInverted((!INVERTED) ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive);
    }

    private void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }

    public Command shootFuel() {
        return runEnd(() -> {
            leftMotor.setDutyCycle(DEFAULT_DUTY_CYCLE);
            rightMotor.setDutyCycle(DEFAULT_DUTY_CYCLE);
        }, () -> stop());
    }
}

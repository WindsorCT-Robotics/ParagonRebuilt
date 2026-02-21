package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

        setInverted(leftMotor, INVERTED);
        setInverted(rightMotor, !INVERTED);
    }

    private void setInverted(ShooterMotorBasic motor, boolean isInverted) {
        motor.configure(nativeMotor -> {
            TalonFXConfiguration config = new TalonFXConfiguration();

            // TODO: What does refresh do? AI says that it prevents overwritting the
            // configuration. Maybe this helps:
            // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/status-signals.html
            nativeMotor.getConfigurator().refresh(config.MotorOutput);

            config.MotorOutput.Inverted = isInverted ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;

            nativeMotor.getConfigurator().apply(config);
        });
    }

    private void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }

    // TODO: Make this a target Rotation Per Second instead of a duty cycle
    public Command shootFuel() {
        return runEnd(() -> {
            leftMotor.setDutyCycle(DEFAULT_DUTY_CYCLE);
            rightMotor.setDutyCycle(DEFAULT_DUTY_CYCLE);
        }, () -> stop());
    }
}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Watts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.CanId;
import frc.robot.hardware.motors.KickerMotor;

public class Kicker extends SubsystemBase {
    private final KickerMotor motor = new KickerMotor(
            "Motor",
            new CanId((byte) 17),
            new TalonFXConfiguration()
                    .withMotorOutput(new MotorOutputConfigs()
                            .withInverted(InvertedValue.CounterClockwise_Positive)
                            .withNeutralMode(NeutralModeValue.Brake))
                    .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(60)))
                    .withMotionMagic(new MotionMagicConfigs()
                            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(2000)))
                    .withSlot0(new Slot0Configs()
                            .withKS(0.03)
                            .withKV(0.01)));

    private static final AngularVelocity PREP_ANGULAR_VELOCITY = RPM.of(1500);
    private AngularVelocity kickVelocity = RotationsPerSecond.of(0);

    public Kicker(String name) {
        super("Subsystems/" + name);
        addChild(motor.getClass().getName(), motor);
        initSmartDashboard();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(
                "Motor Velocity (RPM)",
                () -> getTargetVelocity().in(RPM),
                this::setTargetVelocity);
        builder.addDoubleProperty("Power (Watts)", () -> motor.getPower().in(Watts), null);
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getSubsystem(), this);
        SmartDashboard.putData(getSubsystem() + "/" + motor.getSmartDashboardName(), motor);
    }

    public Command smartDashboardKickFuel() {
        return runEnd(() -> motor.setPointVelocity(getTargetVelocity()), this::hardStop);
    }

    private void hardStop() {
        motor.stop();
    }

    private void stop() {
        motor.setPointVelocity(RPM.zero());
    }

    public Command kickFuel(Supplier<AngularVelocity> velocity) {
        return runEnd(() -> motor.setPointVelocity(velocity.get()), this::hardStop);

    }

    public Command prepareFuel() {
        return runEnd(() -> {
            motor.setPointVelocity(PREP_ANGULAR_VELOCITY);
        }, this::stop);
    }

    private AngularVelocity getTargetVelocity() {
        return kickVelocity;
    }

    private void setTargetVelocity(double rpm) {
        kickVelocity = RPM.of(rpm);
    }
}

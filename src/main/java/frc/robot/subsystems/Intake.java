package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Watts;

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
import frc.robot.hardware.IntakeMotorState;
import frc.robot.hardware.motors.IntakeRollerMotor;

public class Intake extends SubsystemBase {
    private final IntakeRollerMotor motor = new IntakeRollerMotor(
            "Motor",
            new CanId((byte) 16),
            new TalonFXConfiguration()
                    .withMotorOutput(new MotorOutputConfigs()
                            .withInverted(InvertedValue.CounterClockwise_Positive)
                            .withNeutralMode(NeutralModeValue.Brake))
                    .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(90)))
                    .withMotionMagic(new MotionMagicConfigs()
                            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(2000)))
                    .withSlot0(new Slot0Configs()
                            .withKS(0.001)
                            .withKV(0.0105)));

    private final static AngularVelocity AGITATION_VELOCITY = RPM.of(4000);
    private AngularVelocity intakeVelocity = RPM.of(4800);
    private AngularVelocity shuttleVelocity = RPM.of(-4800);

    public Intake(String name) {
        super("Subsystems/" + name);
        addChild(motor.getClass().getName(), motor);

        initSmartDashboard();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Intake Target Velocity",
                () -> getIntakeTargetVelocity().in(RotationsPerSecond),
                this::setIntakeTargetVelocity);

        builder.addDoubleProperty("Shuttle Target Velocity",
                () -> getShuttleTargetVelocity().in(RotationsPerSecond),
                this::setShuttleTargetVelocity);

        builder.addDoubleProperty("Power (Watts)", () -> motor.getPower().in(Watts), null);
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getSubsystem(), this);
        SmartDashboard.putData(getSubsystem() + "/" + motor.getSmartDashboardName(), motor);
    }

    public Command intakeFuel() {
        return runEnd(() -> motor.setPointVelocity(getIntakeTargetVelocity()), this::softStop)
                .withName(getSubsystem() + "/intakeFuel");
    }

    public Command shuttleFuel() {
        return runEnd(() -> motor.setPointVelocity(getShuttleTargetVelocity()), this::softStop)
                .withName(getSubsystem() + "/shuttleFuel");
    }

    public Command stopIntake() {
        return runOnce(() -> {
            softStop();
            motor.setState(IntakeMotorState.IDLE);
        })
                .withName(getSubsystem() + "/stopIntake");
    }

    private void softStop() {
        motor.setPointVelocity(RotationsPerSecond.zero());
    }

    public Command agitateFuel() {
        return runEnd(() -> motor.setPointVelocity(AGITATION_VELOCITY), this::stopIntake);
    }

    private AngularVelocity getIntakeTargetVelocity() {
        return intakeVelocity;
    }

    private void setIntakeTargetVelocity(double RPS) {
        intakeVelocity = RotationsPerSecond.of(RPS);
    }

    private AngularVelocity getShuttleTargetVelocity() {
        return shuttleVelocity;
    }

    private void setShuttleTargetVelocity(double RPS) {
        shuttleVelocity = RotationsPerSecond.of(-RPS);
    }
}
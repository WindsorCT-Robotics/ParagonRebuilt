package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.interfaces.ISystemDynamics;
import frc.robot.hardware.CanId;
import frc.robot.hardware.motors.KickerMotor;

public class Kicker extends SubsystemBase implements ISystemDynamics<KickerMotor> {
    private final KickerMotor motor;
    private final SysIdRoutine routine;
    private static final AngularVelocity PREP_ANGULAR_VELOCITY = RPM.of(1500);
    private AngularVelocity kickVelocity = RotationsPerSecond.of(0);

    public Kicker(String name, CanId motorId) {
        super("Subsystems/" + name);
        motor = new KickerMotor("Motor", motorId, new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(60)))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(2000)))
                .withSlot0(new Slot0Configs()
                        .withKS(0.03)
                        .withKV(0.01)));

        addChild(motor.getClass().getName(), motor);

        routine = new SysIdRoutine(new Config(),
                new Mechanism(this::setSysIdVoltage, log -> log(log, motor, "Kicker Motor"), this));

        initSmartDashboard();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(
                "Motor Velocity (RPM)",
                () -> getTargetVelocity().in(RPM),
                this::setTargetVelocity);
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

    public Command kickFuelToHub(Supplier<AngularVelocity> velocity, Trigger onAllianceSide) {
        return runEnd(() -> {
            if (onAllianceSide.getAsBoolean()) {
                motor.setPointVelocity(velocity.get());
            } else {
                motor.setPointVelocity(RPM.zero());
            }
        }, this::hardStop);
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

    // region SysId
    private void setSysIdVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void log(SysIdRoutineLog log, KickerMotor motor, String name) {
        log.motor(name).angularPosition(motor.getAngle()).angularVelocity(motor.getVelocity());
    }

    @Override
    public Command sysIdDynamic(Direction direction) {
        return routine.dynamic(direction).withName(getSubsystem() + "/sysIdDynamic");
    }

    @Override
    public Command sysIdQuasistatic(Direction direction) {
        return routine.quasistatic(direction).withName(getSubsystem() + "/sysIdQuasistatic");
    }
    // endregion
}

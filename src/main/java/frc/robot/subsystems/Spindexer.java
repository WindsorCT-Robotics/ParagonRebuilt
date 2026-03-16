package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.hardware.CanId;
import frc.robot.hardware.motors.SpindexterMotor;
import frc.robot.interfaces.ISystemDynamics;

public class Spindexer extends SubsystemBase implements ISystemDynamics<SpindexterMotor> {
    private final SpindexterMotor motor;
    private final SysIdRoutine routine;

    private AngularVelocity indexVelocity = RotationsPerSecond.of(0);

    public Spindexer(
            String name,
            CanId motorCanId) {
        super("Subsystems/" + name);
        motor = new SpindexterMotor("Motor", motorCanId, new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(900)))
                .withSlot0(new Slot0Configs()
                        .withKS(0.00325)
                        .withKV(0.011)));

        addChild(this.getName(), motor);

        routine = new SysIdRoutine(new Config(),
                new Mechanism(this::setSysIdVoltage, log -> log(log, motor, "Spindexer Motor"), this));
        initSmartDashboard();
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + motor.getSmartDashboardName(), motor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Indexing Target Velocity (RPM)", () -> getIndexTargetVelocity().in(RPM),
                this::setIndexTargetVelocity);
    }

    private void hardStop() {
        motor.stop();
    }

    private void stop() {
        motor.setPointVelocity(RPM.zero());
    }

    public Command prepareFuel() {
        Timer time = new Timer();
        setIndexTargetVelocity(RPM.of(400).in(RPM));
        return runEnd(() -> {
            if (time.advanceIfElapsed(0.5)) {
                setIndexTargetVelocity(getIndexTargetVelocity().unaryMinus().in(RPM));
                time.restart();
            }
        }, this::stop);
    }

    public Command indexFuel(Supplier<AngularVelocity> velocity) {
        return runEnd(() -> motor.setPointVelocity(velocity.get()), () -> motor.stop());
    }

    public Command indexFueltoHub(Supplier<AngularVelocity> velocity, Trigger isAligned) {
        return runEnd(() -> {
            if (isAligned.getAsBoolean()) {
                motor.setPointVelocity(velocity.get());
            } else {
                hardStop();
            }
        }, this::stop);
    }

    public Command smartDashboardShuttleFuel() {
        return runEnd(() -> motor.setPointVelocity(getIndexTargetVelocity().unaryMinus()), () -> motor.stop());
    }

    public Command smartDashboardIndexFuel() {
        return runEnd(() -> motor.setPointVelocity(getIndexTargetVelocity()), () -> motor.stop());
    }

    public Command indexFuelAtFlyWheelVelocityToHub(
            AngularVelocity indexTargetVelocity,
            Supplier<AngularVelocity> flyWheelVelocity,
            AngularVelocity threshold,
            Trigger isAligned,
            Trigger unstuckFuel,
            Trigger onAllianceSide) {
        return runEnd(() -> {
            if (!onAllianceSide.getAsBoolean()) {
                motor.setPointVelocity(RPM.zero());
                return;
            }

            if (unstuckFuel.getAsBoolean()) {
                motor.setDutyCycle(Percent.of(-10));
                return;
            }

            if (flyWheelVelocity.get().isNear(flyWheelVelocity.get(), threshold) && isAligned.getAsBoolean()) {
                motor.setPointVelocity(indexTargetVelocity);
            } else {
                hardStop();
            }
        }, this::stop);
    }

    public Command indexFuelAtFlyWheelVelocity(
            Supplier<AngularVelocity> indexTargetVelocity,
            Supplier<AngularVelocity> flyWheelVelocity,
            AngularVelocity threshold) {
        return runEnd(() -> {
            if (flyWheelVelocity.get().isNear(flyWheelVelocity.get(), threshold)) {
                motor.setPointVelocity(indexTargetVelocity.get());
            } else {
                hardStop();
            }
        }, this::stop);
    }

    public AngularVelocity getIndexTargetVelocity() {
        return indexVelocity;
    }

    public void setIndexTargetVelocity(double rpm) {
        indexVelocity = RPM.of(rpm);
    }

    // region SysId
    private void setSysIdVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void log(SysIdRoutineLog log, SpindexterMotor motor, String name) {
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
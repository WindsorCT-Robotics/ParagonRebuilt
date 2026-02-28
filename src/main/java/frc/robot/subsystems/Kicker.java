package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.hardware.shooter_motors.KickerMotor;
import frc.robot.interfaces.ISystemDynamics;
import frc.robot.hardware.CanId;

public class Kicker extends SubsystemBase implements ISystemDynamics<KickerMotor> {
    private final KickerMotor motor;
    private final SysIdRoutine routine;
    // TODO: Determine RPS.
    private AngularVelocity kickVelocity = RotationsPerSecond.of(0);
    private final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(0)
            .withMotionMagicExpo_kV(0.12)
            .withMotionMagicExpo_kA(0.1);
    private static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0.25)
            .withKV(0.1225)
            .withKA(300)
            .withKG(0)
            .withGravityType(GravityTypeValue.Elevator_Static);

    public Kicker(String name, CanId motorId) {
        super("Subsystems/" + name);
        motor = new KickerMotor("Motor", motorId, this::setDutyCycle, this::setVoltage);

        motor.configure(motor -> {
            TalonFXConfigurator configurator = motor.getConfigurator();
            configurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
            configurator.apply(MOTION_MAGIC_CONFIGS);
            configurator.apply(SLOT0_CONFIGS);
        });

        addChild(motor.getClass().getName(), motor);

        routine = new SysIdRoutine(new Config(),
                new Mechanism(this::setSysIdVoltage, log -> log(log, motor, "Kicker Motor"), this));

        initSmartDashboard();
    }

    public Command kickStartFuel() {
        return runEnd(() -> motor.setPointVelocity(getTargetVelocity()), () -> motor.stop());
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getSubsystem(), this);
        SmartDashboard.putData(getSubsystem() + "/" + motor.getSmartDashboardName(), motor);
    }

    private AngularVelocity getTargetVelocity() {
        return kickVelocity;
    }

    private void setTargetVelocity(double RPS) {
        kickVelocity = RotationsPerSecond.of(RPS);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Motor Velocity (RPS)", () -> getTargetVelocity().in(RotationsPerSecond),
                this::setTargetVelocity);
    }

    // region SysId
    private void setVoltage(Voltage voltage) {
        CommandScheduler.getInstance().schedule(overrideMotorVoltage(voltage));
    }

    public Command overrideMotorVoltage(Voltage voltage) {
        return runEnd(() -> motor.setVoltage(voltage), () -> motor.stop());
    }

    private void setSysIdVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    private void setDutyCycle(Dimensionless dutyCycle) {
        CommandScheduler.getInstance().schedule(overrideMotorDutyCycle(dutyCycle));
    }

    public Command overrideMotorDutyCycle(Dimensionless dutyCycle) {
        return runEnd(() -> {
            motor.setDutyCycle(dutyCycle);
        }, () -> motor.stop()).withName(getSubsystem() + "/overrideMotorDutyCycle");
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

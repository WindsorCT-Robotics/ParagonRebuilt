package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.hardware.CanId;
import frc.robot.hardware.motors.IntakeRollerMotor;
import frc.robot.interfaces.ISystemDynamics;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Intake extends SubsystemBase implements ISystemDynamics<IntakeRollerMotor> {
    private final IntakeRollerMotor motor;
    private final SysIdRoutine routine;

    private final static AngularVelocity AGITATION_VELOCITY = RPM.of(1000); // TODO: Determine agitation speed.
    private AngularVelocity intakeVelocity = RPM.of(6000);
    private AngularVelocity shuttleVelocity = RPM.of(-4800);

    public Intake(String name, CanId motorCanId) {
        super("Subsystems/" + name);
        motor = new IntakeRollerMotor("Motor", motorCanId, new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(2000)))
                .withSlot0(new Slot0Configs()
                        .withKS(0.001)
                        .withKV(0.0105)));

        addChild(motor.getClass().getName(), motor);

        routine = new SysIdRoutine(
                new Config(),
                new Mechanism(this::setSysIdVoltage,
                        log -> log(log, motor, "IntakeRollerMotor"), this));

        initSmartDashboard();
    }

    @Override
    public void periodic() {
        motor.update();
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
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getSubsystem(), this);
        SmartDashboard.putData(getSubsystem() + "/" + motor.getSmartDashboardName(), motor);
    }

    public Command intakeFuel() {
        return runEnd(() -> motor.setPointVelocity(getIntakeTargetVelocity()), this::stopIntake)
                .withName(getSubsystem() + "/intakeFuel");
    }

    public Command shuttleFuel() {
        return runEnd(() -> motor.setPointVelocity(getShuttleTargetVelocity()), this::stopIntake)
                .withName(getSubsystem() + "/shuttleFuel");
    }

    public Command stopIntake() {
        return runOnce(() -> motor.setPointVelocity(RotationsPerSecond.zero()))
                .withName(getSubsystem() + "/stopIntake");
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

    // region SysId
    private void setSysIdVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void log(SysIdRoutineLog log, IntakeRollerMotor motor, String name) {
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
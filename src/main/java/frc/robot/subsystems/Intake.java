package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.hardware.CanId;
import frc.robot.hardware.intake_motors.IntakeRollerMotor;
import frc.robot.interfaces.ISystemDynamics;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Intake extends SubsystemBase implements ISystemDynamics<IntakeRollerMotor> {
    private final IntakeRollerMotor motor;
    private final SysIdRoutine routine;

    private static final boolean INVERTED = true;
    // TODO: Configure these values.
    private static final FeedForwardConfig FEED_FORWARD_CONFIG = new FeedForwardConfig()
            .kS(0)
            .kV(0)
            .kA(0);

    private static final ClosedLoopConfig CLOSED_LOOP_CONFIG = new ClosedLoopConfig()
            .p(0)
            .i(0)
            .d(0);

    private static final MAXMotionConfig MAX_MOTION_CONFIG = new MAXMotionConfig()
            .allowedProfileError(0)
            .cruiseVelocity(0)
            .maxAcceleration(0)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    private static final ResetMode RESET_MODE = ResetMode.kNoResetSafeParameters;
    private static final PersistMode PERSIST_MODE = PersistMode.kPersistParameters;

    // TODO: Determine RPS.
    private static final AngularVelocity INTAKE_FUEL_VELOCITY = RotationsPerSecond.of(0);
    private static final AngularVelocity SHUTTLE_FUEL_VELOCITY = RotationsPerSecond.of(0);

    public Intake(String name, CanId motorCanId) {
        super("Subsystems/" + name);
        motor = new IntakeRollerMotor("Motor", motorCanId, this::setDutyCycle, this::setVoltage);

        motor.configure(motor -> {
            SparkBaseConfig config = new SparkMaxConfig().inverted(INVERTED);
            config.closedLoop.feedForward.apply(FEED_FORWARD_CONFIG);
            config.closedLoop.apply(CLOSED_LOOP_CONFIG);
            config.closedLoop.maxMotion.apply(MAX_MOTION_CONFIG);

            motor.configure(config,
                    RESET_MODE,
                    PERSIST_MODE);
        });
        addChild(motor.getClass().getName(), motor);

        routine = new SysIdRoutine(
                new Config(),
                new Mechanism(this::setSysIdVoltage,
                        log -> log(log, motor, "IntakeRollerMotor"), this));

        initSmartDashboard();
    }

    public Command intakeFuel() {
        return runOnce(() -> motor.setPointVelocity(INTAKE_FUEL_VELOCITY)).withName(getSubsystem() + "/intakeFuel");
    }

    public Command shuttleFuel() {
        return runOnce(() -> motor.setPointVelocity(SHUTTLE_FUEL_VELOCITY)).withName(getSubsystem() + "/shuttleFuel");
    }

    public Command stopIntake() {
        return run(() -> motor.stop()).withName(getSubsystem() + "/stopIntake");
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getSubsystem(), this);
        SmartDashboard.putData(getSubsystem() + "/" + motor.getSmartDashboardName(), motor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    // region SysId
    private void setDutyCycle(Dimensionless dutyCycle) {
        CommandScheduler.getInstance().schedule(overrideMotorDutyCycle(dutyCycle));
    }

    private void setVoltage(Voltage v) {
        CommandScheduler.getInstance().schedule(overrideMotorVoltage(v));
    }

    private void setSysIdVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    public Command overrideMotorDutyCycle(Dimensionless dutyCycle) {
        return runEnd(() -> {
            motor.setDutyCycle(dutyCycle);
        }, () -> motor.stop()).withName(getSubsystem() + "/overrideMotorDutyCycle");
    }

    public Command overrideMotorVoltage(Voltage v) {
        return runEnd(() -> {
            motor.setVoltage(v);
        }, () -> motor.stop()).withName(getSubsystem() + "/overrideMotorVoltage");
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
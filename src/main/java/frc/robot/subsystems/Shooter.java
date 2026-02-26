package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.hardware.basic_implementations.shooter_motors.ShooterMotorBasic;
import frc.robot.interfaces.ISystemDynamics;
import frc.robot.hardware.CanId;

public class Shooter extends SubsystemBase implements ISystemDynamics<ShooterMotorBasic> {
    private final ShooterMotorBasic leftMotor;
    private final ShooterMotorBasic rightMotor;

    // SysId Routines.
    private final SysIdRoutine routine;
    private static final boolean INVERTED = false;
    private static final Dimensionless DEFAULT_DUTY_CYCLE = Percent.of(10);

    public Shooter(String name, CanId leftMotorId, CanId rightMotorId) {
        super("Subsystems/" + name);
        leftMotor = new ShooterMotorBasic("Left Shooter Motor", leftMotorId, this::setDutyCycle, this::setVoltage);
        rightMotor = new ShooterMotorBasic("Right Shooter Motor", rightMotorId, this::setDutyCycle, this::setVoltage);

        setInverted(leftMotor, INVERTED);
        setInverted(rightMotor, !INVERTED);

        addChild(this.getName(), leftMotor);
        addChild(this.getName(), rightMotor);
        // TODO: Consider customizing new Config(). Should be customized if motor has
        // physical limitations.
        routine = new SysIdRoutine(new Config(), new Mechanism(this::setSysIdVoltage, log -> {
            log(log, leftMotor, "Left Shooter Motor");
            log(log, rightMotor, "Right Shooter Motor");
        }, this));
        initSmartDashboard();
    }

    // TODO: Make this a target Rotation Per Second instead of a duty cycle
    public Command shootFuel() {
        return runEnd(() -> {
            leftMotor.setDutyCycle(DEFAULT_DUTY_CYCLE);
            rightMotor.setDutyCycle(DEFAULT_DUTY_CYCLE);
        }, this::stop);
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/Left " + leftMotor.getClass().getSimpleName(), leftMotor);
        SmartDashboard.putData(getName() + "/Right " + rightMotor.getClass().getSimpleName(),
                rightMotor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
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

    private void setSysIdVoltage(Voltage voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    private void setVoltage(Voltage voltage) {
        CommandScheduler.getInstance().schedule(overrideMotorVoltage(voltage));
    }

    public Command overrideMotorVoltage(Voltage voltage) {
        return runEnd(() -> {
            leftMotor.setVoltage(voltage);
            rightMotor.setVoltage(voltage);
        }, this::stop);
    }

    private void setDutyCycle(Dimensionless dutyCycle) {
        CommandScheduler.getInstance().schedule(overrideMotorDutyCycle(dutyCycle));
    }

    public Command overrideMotorDutyCycle(Dimensionless dutyCycle) {
        return runEnd(() -> {
            leftMotor.setDutyCycle(dutyCycle);
            rightMotor.setDutyCycle(dutyCycle);
        }, this::stop).withName(getSubsystem() + "/overrideMotorDutyCycle");
    }

    @Override
    public void log(SysIdRoutineLog log, ShooterMotorBasic motor, String name) {
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
}

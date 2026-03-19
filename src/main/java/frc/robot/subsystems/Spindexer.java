package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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
import frc.robot.hardware.sensors.FuelSensor;
import frc.robot.interfaces.ISystemDynamics;

public class Spindexer extends SubsystemBase implements ISystemDynamics<SpindexterMotor> {
    private final SpindexterMotor motor;
    private final FuelSensor fuelSensor;
    private final SysIdRoutine routine;

    private AngularVelocity indexVelocity = RPM.of(2000);
    private static final AngularVelocity UNSTUCK_VELOCITY = RPM.of(-800);
    private static final Distance FUEL_SENSOR_THRESHOLD = Millimeters.of(50);
    private final Trigger stuckRoutine;
    private final Timer stuckRoutineTimer = new Timer();
    private boolean initStuckTimer = false;

    private boolean indexingToScore = false;

    public Spindexer(
            String subsystemName,
            String fuelSensorName,
            CanId motorCanId,
            CanId fuelSensorCanId) {
        super("Subsystems/" + subsystemName);
        motor = new SpindexterMotor("Motor", motorCanId, new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(900)))
                .withSlot0(new Slot0Configs()
                        .withKS(0.00325)
                        .withKV(0.011)));

        fuelSensor = new FuelSensor(
                fuelSensorName,
                fuelSensorCanId,
                FUEL_SENSOR_THRESHOLD,
                RangingMode.Short,
                Milliseconds.of(20),
                this::getIndexingToScore);

        stuckRoutine = new Trigger(() -> indexingToScore
                && fuelSensor.getElapsedSinceNofuel().gt(Seconds.of(2)));

        addChild(this.getName(), motor);

        routine = new SysIdRoutine(new Config(),
                new Mechanism(this::setSysIdVoltage, log -> log(log, motor, "Spindexer Motor"), this));
        initSmartDashboard();
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + motor.getSmartDashboardName(), motor);
        SmartDashboard.putData(getName() + "/" + fuelSensor.getSmartDashboardName(), fuelSensor);
    }

    @Override
    public void periodic() {
        fuelSensor.update();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(
                "Indexing Target Velocity (RPM)",
                () -> getIndexTargetVelocity().in(RPM),
                this::setIndexTargetVelocity);
    }

    private boolean getIndexingToScore() {
        return indexingToScore;
    }

    private void hardStop() {
        motor.stop();
        indexingToScore = false;
    }

    private void stop() {
        motor.setPointVelocity(RPM.zero());
        indexingToScore = false;
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
            Supplier<AngularVelocity> indexTargetVelocity,
            Trigger launcherAtTargetRPM,
            Trigger overrideLauncherAtTargetRPM,
            Trigger isAligned,
            Trigger manualUnstuckFuel,
            Trigger onAllianceSide) {
        return runEnd(() -> {
            if (!onAllianceSide.getAsBoolean()) {
                motor.setPointVelocity(RPM.zero());
                indexingToScore = false;
                return;
            }

            if (manualUnstuckFuel.getAsBoolean()) {
                motor.setPointVelocity(UNSTUCK_VELOCITY);
                indexingToScore = false;
                return;
            }

            if (!initStuckTimer && stuckRoutine.getAsBoolean()) {
                stuckRoutineTimer.restart();
                initStuckTimer = true;
            }

            if (initStuckTimer) {
                if (!stuckRoutineTimer.hasElapsed(Seconds.of(0.5))) {
                    motor.setPointVelocity(UNSTUCK_VELOCITY);
                    indexingToScore = false; // Shouldn't set to false here, since reversing spindexer is still a part of current scoring cycle
                    return;
                } else {
                    initStuckTimer = false;
                }
            }

            if (launcherAtTargetRPM.and(isAligned).getAsBoolean()
                    || overrideLauncherAtTargetRPM.getAsBoolean()) {
                motor.setPointVelocity(indexTargetVelocity.get());
                indexingToScore = true;
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

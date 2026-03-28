package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    private static final AngularVelocity INDEX_FUEL_VELOCITY = RPM.of(6000);
    private static final AngularVelocity AGITATE_FUEL_VELOCITY = RPM.of(-800);
    private AngularVelocity smartDashboardVelocity = RPM.of(0);

    private static final Distance FUEL_SENSOR_THRESHOLD = Millimeters.of(50);
    public final Trigger autoUnjamTrigger;

    private boolean indexingToScore = false;

    public Spindexer(
            String subsystemName,
            CanId motorCanId,
            CanId fuelSensorCanId) {
        super("Subsystems/" + subsystemName);
        motor = new SpindexterMotor("Motor", motorCanId, new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(80)))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(900)))
                .withSlot0(new Slot0Configs()
                        .withKP(0.03)
                        .withKS(0.03)
                        .withKV(0.009)));

        fuelSensor = new FuelSensor(
                "Fuel Sensor",
                fuelSensorCanId,
                FUEL_SENSOR_THRESHOLD,
                RangingMode.Short,
                Milliseconds.of(20),
                this::getIndexingToScore);

        autoUnjamTrigger = new Trigger(() -> indexingToScore
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

    private AngularVelocity getSmartdashBoardVelocity() {
        return smartDashboardVelocity;
    }

    private void setSmartdashBoardVelocity(double rpm) {
        smartDashboardVelocity = RPM.of(rpm);
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
                () -> getSmartdashBoardVelocity().in(RPM),
                this::setSmartdashBoardVelocity);
        builder.addBooleanProperty("Unjam Fuel", autoUnjamTrigger, null);
    }

    private boolean getIndexingToScore() {
        return indexingToScore;
    }

    private void stop() {
        motor.stop();
        indexingToScore = false;
    }

    public Command indexFuel() {
        return runEnd(() -> {
            motor.setPointVelocity(INDEX_FUEL_VELOCITY);
            indexingToScore = true;
        }, this::stop);
    }

    public Command agitateFuel() {
        return run(() -> motor.setPointVelocity(AGITATE_FUEL_VELOCITY))
                .raceWith(new WaitCommand(Seconds.of(0.5))
                        .finallyDo(this::stop));
    }

    public Command manualAgitateFuel() {
        return runEnd(() -> motor.setPointVelocity(AGITATE_FUEL_VELOCITY), this::stop);
    }

    public Command smartDashboardIndexFuel() {
        return runEnd(() -> {
            motor.setPointVelocity(getSmartdashBoardVelocity());
            indexingToScore = true;
        }, this::stop);
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
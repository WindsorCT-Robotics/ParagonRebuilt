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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.hardware.CanId;
import frc.robot.hardware.motors.SpindexterMotor;
import frc.robot.hardware.sensors.FuelSensor;

public class Spindexer extends SubsystemBase {
    private final SpindexterMotor motor = new SpindexterMotor(
            "Motor",
            new CanId((byte) 13),
            new TalonFXConfiguration()
                    .withMotorOutput(new MotorOutputConfigs()
                            .withInverted(InvertedValue.CounterClockwise_Positive)
                            .withNeutralMode(NeutralModeValue.Brake))
                    .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(60)))
                    .withMotionMagic(new MotionMagicConfigs()
                            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(900)))
                    .withSlot0(new Slot0Configs()
                            .withKP(0.03)
                            .withKS(0.03)
                            .withKV(0.009)));
    private final FuelSensor fuelSensor = new FuelSensor(
            "Fuel Sensor",
            new CanId((byte) 13),
            FUEL_SENSOR_THRESHOLD,
            RangingMode.Short,
            Milliseconds.of(20),
            this::getIndexingToScore);

    private static final AngularVelocity INDEX_FUEL_VELOCITY = RPM.of(3500);
    private static final AngularVelocity PREPARE_FUEL_VELOCITY = RPM.of(-400);
    private static final AngularVelocity AGITATE_FUEL_VELOCITY = RPM.of(-800);
    private AngularVelocity smartDashboardVelocity = RPM.of(0);

    private static final Distance FUEL_SENSOR_THRESHOLD = Millimeters.of(50);

    private boolean indexingToScore = false;

    public Spindexer(String name) {
        super("Subsystems/" + name);

        addChild(this.getName(), motor);

        initSmartDashboard();
    }

    private void initSmartDashboard() {
        SmartDashboard.putData(getName(), this);
        SmartDashboard.putData(getName() + "/" + motor.getSmartDashboardName(), motor);
        SmartDashboard.putData(getName() + "/" + fuelSensor.getSmartDashboardName(), fuelSensor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(
                "Indexing Target Velocity (RPM)",
                () -> getSmartdashBoardVelocity().in(RPM),
                this::setSmartdashBoardVelocity);
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

    public Command indexFuelAlgorithim() {
        return new WaitCommand(Seconds.of(2)).deadlineFor(indexFuel())
                .andThen(new WaitCommand(Seconds.of(0.5)).deadlineFor(
                        runEnd(() -> motor.setPointVelocity(INDEX_FUEL_VELOCITY.div(1.5)), this::stop))
                        .repeatedly());
    }

    public Command agitateFuel() {
        return runEnd(() -> motor.setPointVelocity(AGITATE_FUEL_VELOCITY), this::stop);
    }

    public Command prepareFuel() {
        return runEnd(() -> {
            motor.setPointVelocity(PREPARE_FUEL_VELOCITY);
        }, this::stop);
    }

    public Command smartDashboardIndexFuel() {
        return runEnd(() -> {
            motor.setPointVelocity(getSmartdashBoardVelocity());
            indexingToScore = true;
        }, this::stop);
    }
}
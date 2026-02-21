package frc.robot.hardware.basic_implementations.intake_motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.hardware.CanId;
import frc.robot.hardware.DigitalInputOutput;
import frc.robot.hardware.base_motors.NeoMotorBase;
import frc.robot.interfaces.IHomingMotor;

public class BayDoorMotorBasic extends NeoMotorBase implements IHomingMotor<SparkMax, SparkMaxConfigAccessor> {
    private static final Current STRUGGLE_THRESHOLD = Amps.of(40); // TODO: Test value
    private static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 0); // TODO: Figure
                                                                                                  // velocity (kV)

    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(1); // TODO: Figure good speed.
    // These are zero because the Bay Door should only be controlled by setting the
    // rotations per second.
    private static final Voltage MAX_VOLTAGE = Volts.of(0);
    private static final Dimensionless MAX_PERCENTAGE = Percent.of(0.05);
    private static final Dimensionless HOMING_DUTY_CYCLE = Percent.of(-0.1);
    private final DigitalInput limit;
    private boolean homingComplete = false;

    private BayDoorState motorBayDoorState = BayDoorState.UNKNOWN;

    public BayDoorMotorBasic(String name, CanId canId, DigitalInput limit) {
        super(name, canId, feedforward,
                new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(
                        (int) DEFAULT_CURRENT.in(Amps)),
                // https://docs.revrobotics.com/revlib/configuring-devices#resetting-parameters-before-configuring
                ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters,
                MAX_ANGULAR_VELOCITY, MAX_VOLTAGE, MAX_PERCENTAGE);

        this.limit = limit;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("Bay Motor State", () -> getBayMotorState().toString(), null);
        builder.addBooleanProperty("Is Moving", this::isMoving, null);
        builder.addBooleanProperty("Is Struggling", this::isStruggling, null);
    }

    public BayDoorState getBayMotorState() {
        return motorBayDoorState;
    }

    public void setBayMotorState(BayDoorState state) {
        motorBayDoorState = state;
    }

    public boolean isStruggling() {
        return getCurrent().gt(STRUGGLE_THRESHOLD);
    }

    @Override
    public boolean isHomed() {
        return homingComplete;
    }

    @Override
    public void home() {
        if (!limit.get()) {
            motor.set(HOMING_DUTY_CYCLE.in(Value));
        }
        else {
            motor.stopMotor();
            resetRelativeEncoder();
            homingComplete = true;
        }
    }
}
package frc.robot.hardware.basic_implementations.intake_motors;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.Consumer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.NeoMotorBase;
import frc.robot.interfaces.IHomingMotor;

public class BayDoorMotorBasic extends NeoMotorBase implements IHomingMotor<SparkMax, SparkMaxConfigAccessor> {
    private final DigitalInput limit;
    private boolean homingComplete = false;

    private BayDoorState motorBayDoorState = BayDoorState.UNKNOWN;

    public BayDoorMotorBasic(
            String name,
            CanId canId,
            DigitalInput limit,
            Consumer<Dimensionless> dutyCycleSetter,
            Consumer<Voltage> voltageSetter) {
        super(
                name,
                canId,
                new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(
                        (int) DEFAULT_CURRENT.in(Amps)),
                // https://docs.revrobotics.com/revlib/configuring-devices#resetting-parameters-before-configuring
                ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters,
                dutyCycleSetter,
                voltageSetter);

        this.limit = limit;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("Bay Motor State", () -> getBayMotorState().toString(), null);
        builder.addBooleanProperty("Is Moving", this::isMoving, null);
    }

    public BayDoorState getBayMotorState() {
        return motorBayDoorState;
    }

    public void setBayMotorState(BayDoorState state) {
        motorBayDoorState = state;
    }

    @Override
    public boolean isHomed() {
        return homingComplete;
    }

    @Override
    public void home(Dimensionless dutyCycle) {
        if (!limit.get()) {
            setDutyCycle(dutyCycle);
        } else {
            stop();
            resetRelativeEncoder();
            setBayMotorState(BayDoorState.CLOSE);
            homingComplete = true;
        }
    }

    public void homeAsFollower(Dimensionless dutyCycle) {
        if (!limit.get()) {
            pauseFollower();
            setDutyCycle(dutyCycle);
        } else {
            stop();
            resetRelativeEncoder();
            setBayMotorState(BayDoorState.CLOSE);
            resumeFollower();
            homingComplete = true;
        }
    }
}
package frc.robot.hardware.basic_implementations.intake_motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class BayDoorMotorBasic extends KrakenMotorBase {
    private BayMotorState motorBayDoorState = BayMotorState.UNKNOWN;
    private boolean hasHomed = false;

    public BayDoorMotorBasic(
            String name,
            CanId canId,
            TalonFXConfiguration configuration) {
        super(
                name,
                canId,
                configuration);

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("Bay Motor State", () -> getBayMotorState().toString(), null);
        builder.addBooleanProperty("Is Moving", this::isMoving, null);
    }

    public BayMotorState getBayMotorState() {
        return motorBayDoorState;
    }

    public void setBayMotorState(BayMotorState state) {
        motorBayDoorState = state;
    }

    public boolean hasHomed() {
        return hasHomed;
    }

    public void home(boolean stopHoming, Dimensionless homeDutyCycle) {
        if (!stopHoming) {
            setDutyCycle(homeDutyCycle);
        } else {
            stop();
            hasHomed = true;
            setBayMotorState(BayMotorState.CLOSE);
        }
        setBayMotorState(BayMotorState.CLOSING);
    }
}
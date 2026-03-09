package frc.robot.hardware.basic_implementations.intake_motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.hardware.CanId;
import frc.robot.hardware.base_motors.KrakenMotorBase;

public class BayDoorMotorBasic extends KrakenMotorBase {
    private BayDoorState motorBayDoorState = BayDoorState.UNKNOWN;

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

    public BayDoorState getBayMotorState() {
        return motorBayDoorState;
    }

    public void setBayMotorState(BayDoorState state) {
        motorBayDoorState = state;
    }
}
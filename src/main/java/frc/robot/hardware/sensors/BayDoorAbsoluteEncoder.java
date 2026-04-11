package frc.robot.hardware.sensors;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import frc.robot.hardware.CanId;
import frc.robot.hardware.base_sensors.CanCoderBase;

public class BayDoorAbsoluteEncoder extends CanCoderBase {
    public BayDoorAbsoluteEncoder(
            String name,
            CanId canId,
            CANcoderConfiguration configuration) {
        super(name, canId, configuration);
    }
}
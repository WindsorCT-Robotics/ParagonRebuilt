package frc.robot.hardware.base_sensors;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.hardware.CanId;
import frc.robot.interfaces.IAbsoluteEncoder;
import frc.robot.interfaces.IConfiguration;

public class CanCoderBase implements IAbsoluteEncoder, IConfiguration<CANcoderConfiguration>, Sendable {
    protected final CANcoder canCoder;
    protected final CANcoderConfigurator configurator;
    private final CANcoderConfiguration defaultConfiguration;
    private CANcoderConfiguration currentConfiguration;
    private final CanId canId;
    private final String name;

    public CanCoderBase(
            String name,
            CanId canId,
            CANcoderConfiguration configuration) {
        canCoder = new CANcoder(canId.Id());
        configurator = canCoder.getConfigurator();
        defaultConfiguration = configuration;
        currentConfiguration = configuration;
        configurator.apply(defaultConfiguration);
        this.canId = canId;
        this.name = name;
    }

    @Override
    public Angle getAbsolutePosition() {
        return canCoder.getAbsolutePosition().getValue();
        // return canCoder.getPosition().getValue();
    }

    @Override
    public CanId getId() {
        return canId;
    }

    @Override
    public Angle getRelativePosition() {
        return canCoder.getPositionSinceBoot().getValue();
    }

    @Override
    public AngularVelocity getVelocity() {
        return canCoder.getVelocity().getValue();
    }

    @Override
    public Voltage getVoltage() {
        return canCoder.getSupplyVoltage().getValue();
    }

    @Override
    public void setPosition(Angle angle) {
        canCoder.setPosition(angle);
    }

    @Override
    public void configure(CANcoderConfiguration configuration) {
        currentConfiguration = configuration;
        configurator.apply(configuration);
    }

    @Override
    public CANcoderConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public CANcoderConfiguration getDefaultConfiguration() {
        return defaultConfiguration;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Absolute Position (Rotations)", () -> getAbsolutePosition().in(Rotations), null);
        builder.addDoubleProperty("Relative Position (Rotations)", () -> getRelativePosition().in(Rotations), null);
        builder.addDoubleProperty("Velocity (RPM)", () -> getVelocity().in(RPM), null);
        builder.addDoubleProperty("Voltage (V)", () -> getVoltage().in(Volts), null);
    }

    public String getSmartDashboardName() {
        return name;
    }
}
package frc.robot.hardware.base_motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.hardware.CanId;
import frc.robot.interfaces.IClosedLoopMotor;

public class TalonFXMotorBase implements IClosedLoopMotor, Sendable {
    protected final TalonFX motor;
    protected final TalonFXConfigurator configurator;
    private final TalonFXConfiguration defaultConfiguration;
    private TalonFXConfiguration currentConfiguration;
    private final String name;

    public TalonFXMotorBase(
            String name,
            CanId canId,
            TalonFXConfiguration configuration) {
        motor = new TalonFX(canId.Id());
        configurator = motor.getConfigurator();
        defaultConfiguration = configuration;
        currentConfiguration = defaultConfiguration;
        configurator.apply(defaultConfiguration);
        this.name = name;
    }

    public void follow(int Id, MotorAlignmentValue alignment) {
        motor.setControl(new Follower(Id, alignment));
    }

    public void followAndIgnoreInversion(int Id) {
        motor.setControl(new StrictFollower(Id));
    }

    @Override
    public String getSmartDashboardName() {
        return name;
    }

    @Override
    public void setPointPosition(Angle angle) {
        motor.setControl(new MotionMagicVoltage(angle));
    }

    @Override
    public void setPointVelocity(AngularVelocity angularVelocity) {
        motor.setControl(new MotionMagicVelocityDutyCycle(angularVelocity));
    }

    public void configure(TalonFXConfiguration configuration) {
        currentConfiguration = configuration;
        configurator.apply(configuration);
    }

    public TalonFXConfiguration getCurrentConfiguration() {
        return currentConfiguration.clone();
    }

    public TalonFXConfiguration getDefaultConfiguration() {
        return defaultConfiguration.clone();
    }

    @Override
    public Angle getAngle() {
        return motor.getPosition().getValue();
    }

    @Override
    public Dimensionless getDutyCycle() {
        return Percent.of(motor.getDutyCycle().getValue());
    }

    /**
     * Duty Cycles are proportional to battery life.
     */
    @Override
    public void setDutyCycle(Dimensionless percentage) {
        motor.set(percentage.in(Value));
    }

    @Override
    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }

    @Override
    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public boolean isMoving() {
        return motor.getVelocity().getValueAsDouble() != 0;
    }

    @Override
    public void resetRelativeEncoder() {
        motor.setPosition(Degrees.zero());
    }

    @Override
    public Current getCurrent() {
        return motor.getTorqueCurrent().getValue();
    }

    @Override
    public Temperature getTemperarure() {
        return motor.getDeviceTemp().getValue();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setActuator(true);
        builder.setSafeState(this::stop);

        builder.addDoubleProperty("Angle (Rotations)", () -> getAngle().in(Rotations), null);
        builder.addDoubleProperty("Velocity (RPM)", () -> getVelocity().in(RPM), null);
        builder.addBooleanProperty("Is Moving", this::isMoving, null);
        builder.addDoubleProperty("Current (Amps)", () -> getCurrent().in(Amps), null);
        builder.addDoubleProperty("Temperature (C)", () -> getTemperarure().in(Celsius), null);
    }
}

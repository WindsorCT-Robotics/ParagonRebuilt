package frc.robot.hardware.intakeMotors;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.interfaces.IAngularPositionMotor;

public class IntakeBayDoorDualMotors implements IAngularPositionMotor, Sendable {
    private final IntakeBayDoorMotor leadMotor;
    private final IntakeBayDoorMotor followerMotor;
    private static final Dimensionless MAX_DUTY = Percent.of(100);
    private static final Dimensionless MIN_DUTY = Percent.of(-100);

    public IntakeBayDoorDualMotors(
            String name,
            IntakeBayDoorMotor leadMotor,
            IntakeBayDoorMotor followerMotor,
            boolean inverted) {
        SendableRegistry.add(this, name);
        this.leadMotor = leadMotor;
        this.followerMotor = followerMotor;
        followerMotor.setFollower(leadMotor);
        setInverted(inverted);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setActuator(true);
        builder.setSafeState(this::stop);

        builder.addBooleanProperty("Is Motor Moving?", this::isMoving, null);
        builder.addDoubleProperty("Voltage (Volts)", () -> getVoltage().in(Volts), null);
        builder.addDoubleProperty("Angle (Degrees)", () -> getAngle().in(Degrees), null);
    }

    @Override
    public void setAngularPosition(Angle angle) {
        leadMotor.setAngularPosition(angle);
    }

    @Override
    public Voltage getVoltage() {
        return leadMotor.getVoltage().plus(leadMotor.getVoltage()).div(2); // Average Between both motors
    }

    @Override
    public boolean isMoving() {
        return leadMotor.isMoving();
    }

    @Override
    public void resetRelativeEncoder() {
        leadMotor.resetRelativeEncoder(); // TODO: Since right motor is following does the right motor encoder reset
                                          // too?
    }

    @Override
    public void setDutyCycle(Dimensionless percentage) {
        leadMotor.setDutyCycle(percentage);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        leadMotor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        leadMotor.stop();
    }

    private Angle getAngle() {
        return leadMotor.getAngle().plus(followerMotor.getAngle()).div(2);
    }

    private void setInverted(boolean inverted) {
        leadMotor.setInverted(inverted);
        followerMotor.setInverted(!inverted);
    }

    public final Trigger isClosed() {
        return leadMotor.isAtReverseLimit;
    }

    public final Trigger isOpen() {
        return leadMotor.isAtReverseLimit;
    }

    public void setIdleMode(IdleMode idleMode) {
        leadMotor.setIdleMode(idleMode);
        followerMotor.setIdleMode(idleMode);
    }
}
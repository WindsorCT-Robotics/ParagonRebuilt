package frc.robot.hardware;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.interfaces.IAngularPositionMotor;

public class IntakeBayDoorDualMotors implements IAngularPositionMotor, Sendable {
    private final IntakeBayDoorMotor leftMotor;
    private final IntakeBayDoorMotor rightMotor;
    private static final Dimensionless MAX_DUTY = Percent.of(100);
    private static final Dimensionless MIN_DUTY = Percent.of(-100);

    public IntakeBayDoorDualMotors(
            String name,
            IntakeBayDoorMotor leftMotor,
            IntakeBayDoorMotor rightMotor,
            boolean inverted) {
        SendableRegistry.add(this, name);
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        rightMotor.setFollower(leftMotor);
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
        leftMotor.setAngularPosition(angle);
    }

    @Override
    public Voltage getVoltage() {
        return leftMotor.getVoltage().plus(rightMotor.getVoltage()).div(2); // Average Between both motors
    }

    @Override
    public boolean isMoving() {
        return leftMotor.isMoving();
    }

    @Override
    public void resetRelativeEncoder() {
        leftMotor.resetRelativeEncoder(); // TODO: Since right motor is following does the right motor encoder reset
                                          // too?
    }

    @Override
    public void setDutyCycle(Dimensionless percentage) {
        leftMotor.setDutyCycle(percentage);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        leftMotor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        leftMotor.stop();
    }

    public Angle getAngle() {
        return Degrees.of(leftMotor.getAngle() + rightMotor.getAngle()).div(2);
    }

    private void setInverted(boolean inverted) {
        leftMotor.setInverted(inverted);
        rightMotor.setInverted(!inverted);
    }
}
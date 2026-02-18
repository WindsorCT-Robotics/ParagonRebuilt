package frc.robot.hardware.basic_implementations.intake_motors;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.CanId;
import frc.robot.interfaces.IMotor;

public class BayDoorDualMotorBasic implements IMotor {
    private final BayDoorMotorBasic leadMotor;
    private final BayDoorMotorBasic followerMotor;
    private final static boolean INVERTED = false;

    public BayDoorDualMotorBasic(
            String name,
            CanId leadId,
            CanId followerId) {
        leadMotor = new BayDoorMotorBasic(name, leadId);
        followerMotor = new BayDoorMotorBasic(name, followerId);
        followerMotor.setFollower(leadMotor);
        leadMotor.setInverted(INVERTED);
        followerMotor.setInverted(!INVERTED);
    }

    @Override
    public Voltage getVoltage() {
        return leadMotor.getVoltage().plus(followerMotor.getVoltage()).div(2);
    }

    @Override
    public boolean isMoving() {
        return leadMotor.isMoving();
    }

    @Override
    public void resetRelativeEncoder() {
        leadMotor.resetRelativeEncoder();
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
}

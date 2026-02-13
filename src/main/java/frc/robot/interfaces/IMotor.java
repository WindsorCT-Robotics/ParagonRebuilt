package frc.robot.interfaces;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;

public interface IMotor {
    public void setVoltage(Voltage voltage);

    public Voltage getVoltage();

    public boolean isMoving();

    public void stop();

    public void resetRelativeEncoder();

    public void setDutyCycle(Dimensionless percentage);
}
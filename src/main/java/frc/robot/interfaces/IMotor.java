package frc.robot.interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface IMotor<Motor> {

    public void stop();

    public void resetRelativeEncoder();

    public void setDutyCycle(Dimensionless percentage);

    public Dimensionless getDutyCycle();

    public void setVoltage(Voltage voltage);

    public Voltage getVoltage();

    public AngularVelocity getVelocity();

    public boolean isMoving();

    public Angle getAngle();

    public Temperature getTemperarure();

    public Current getCurrent();

    public String getSmartDashboardName();
}
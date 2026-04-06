package frc.robot.interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface IMotor {

    public void stop();

    public void resetRelativeEncoder();

    public void setDutyCycle(Dimensionless percentage);

    public Dimensionless getDutyCycle();

    public void setVoltage(Voltage voltage);

    public Voltage getVoltage();

    public AngularVelocity getVelocity();

    public Angle getAngle();

    public Temperature getTemperature();

    public Current getCurrent();

    public Power getPower();

    public String getSmartDashboardName();
}
package frc.robot.interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.CanId;

public interface IAbsoluteEncoder {
    public Angle getAbsolutePosition();
    public Angle getRelativePosition();
    public void setPosition(Angle angle);
    public AngularVelocity getVelocity();
    public CanId getId();
    public Voltage getVoltage();
}

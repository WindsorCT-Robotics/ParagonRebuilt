package frc.robot.interfaces;

import edu.wpi.first.units.measure.Angle;

public interface IAngularPositionMotor extends IMotor {
    public void setAngularPosition(Angle angle);
}

package frc.robot.interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface IClosedLoopMotor<Motor> extends IMotor {
    public void setPointPosition(Angle angle);

    public void setPointVelocity(AngularVelocity angularVelocity);
}

package frc.robot.interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public interface IMaxMotionTuner extends IMotorTuner {
    public void updateAllowedProfileError(double value);

    public Angle getAllowedProfileError();

    public void updateCruiseVelocity(double value);

    public AngularVelocity getCruiseVelocity();

    public void updateMaxAcceleration(double value);

    public AngularAcceleration getMaxAcceleration();
}

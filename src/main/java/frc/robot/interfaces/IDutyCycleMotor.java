package frc.robot.interfaces;

import edu.wpi.first.units.measure.Dimensionless;

public interface IDutyCycleMotor extends IMotor {

    public void setDutyCycle(Dimensionless percentage);
}

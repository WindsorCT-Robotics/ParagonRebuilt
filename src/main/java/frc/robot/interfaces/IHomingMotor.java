package frc.robot.interfaces;

import edu.wpi.first.units.measure.Dimensionless;

public interface IHomingMotor extends IMotor {
    boolean hasHomed();

    void home(boolean stopHoming, Dimensionless dutyCycle);
}

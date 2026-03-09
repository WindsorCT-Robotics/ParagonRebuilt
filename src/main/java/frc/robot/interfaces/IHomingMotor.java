package frc.robot.interfaces;

import edu.wpi.first.units.measure.Dimensionless;

public interface IHomingMotor<Motor> extends IMotor<Motor> {
    boolean hasHomed();

    void home(boolean stopHoming, Dimensionless dutyCycle);
}

package frc.robot.interfaces;

import edu.wpi.first.units.measure.Dimensionless;

public interface IHomingMotor<Motor, Config> extends IMotor<Motor, Config> {
    boolean isHomed();

    void home(Dimensionless dutyCycle);
}

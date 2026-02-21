package frc.robot.interfaces;

public interface IHomingMotor<Motor, Config> extends IMotor<Motor, Config> {
    boolean isHomed();
    void home();
}

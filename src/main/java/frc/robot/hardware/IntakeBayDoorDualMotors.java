package frc.robot.hardware;

public class IntakeBayDoorDualMotors {
    private final IntakeBayDoorMotor leftMotor;
    private final IntakeBayDoorMotor rightMotor;

    protected IntakeBayDoorDualMotors(
            IntakeBayDoorMotor leftMotor,
            IntakeBayDoorMotor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }
}
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.CanId;
import frc.robot.hardware.DigitalInputOutput;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorMotorBasic;

public class BayDoor2 extends SubsystemBase {
    private final BayDoorMotorBasic leftMotor;
    private final BayDoorMotorBasic rightMotor;
    private final DigitalInput leftHardLimit;
    private final DigitalInput rightHardLimit;
    private static final Angle FORWARD_POSITION_TOLERANCE = Rotations.of(1);
    private static final Angle REVERSE_POSITION_TOLERANCE = Rotations.of(1);
    private static final Dimensionless DEFAULT_DUTY_CYCLE = Percent.of(0.1);
    private static final Dimensionless HOME_DUTY_CYCLE = Percent.of(-0.1);
    private static final boolean INVERTED = true;
    private BayDoorState bayDoorState = BayDoorState.UNKNOWN;

    public BayDoor2(
            String name, 
            CanId leftMotorId, 
            CanId rightMotorId, 
            DigitalInputOutput leftLimitSwitchDIO,
            DigitalInputOutput rightLimitSwitchDIO) {
        leftMotor = new BayDoorMotorBasic("Left Motor", leftMotorId);
        rightMotor = new BayDoorMotorBasic("Right Motor", rightMotorId);
        leftHardLimit = new DigitalInput(leftLimitSwitchDIO.Id());
        rightHardLimit = new DigitalInput(rightLimitSwitchDIO.Id());
        leftMotor.setInverted(INVERTED);
        rightMotor.setInverted(!INVERTED);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Bay Door State", bayDoorState.toString());
        SmartDashboard.putBoolean("Is Moving", (leftMotor.isMoving() || rightMotor.isMoving()));
        SmartDashboard.putNumber("Left Motor Rotation", leftMotor.getRotation().in(Rotations));
        SmartDashboard.putNumber("Right Motor Rotation", rightMotor.getRotation().in(Rotations));
    }

    private enum BayDoorState {
        UNKNOWN,
        OPEN,
        OPENING,
        CLOSE,
        CLOSING
    }

    private enum BayDoorAction {
        OPEN,
        CLOSE
    }

    private void moveTowards(BayDoorMotorBasic motor, Angle position) {
        if (motor.getRotation().lt(position)) {
            motor.setDutyCycle(DEFAULT_DUTY_CYCLE);
        } else {
            motor.setDutyCycle(DEFAULT_DUTY_CYCLE.times(-1));
        }
    }

    private void moveToPosition(BayDoorMotorBasic motor, Angle position, Angle goalPosition, Angle tolerance) {
        System.out.println(goalPosition.in(Rotations) + " | " + position.in(Rotations));
        if (!position.isNear(goalPosition, tolerance)) {
            moveTowards(motor, goalPosition);
        } else {
            motor.stop();
        }
    }

    // TODO: Detect if at ends such as OPEN and CLOSE. Currently doesn't set that.
    private Command moveBayDoorTo(BayDoorAction action) {
        Angle goalPosition;
        Angle tolerance;

        switch (action) {
            case OPEN:
                goalPosition = BayDoorMotorBasic.OPEN_ANGLE;
                bayDoorState = BayDoorState.OPENING;
                tolerance = FORWARD_POSITION_TOLERANCE;
                break;
            case CLOSE:
                goalPosition = BayDoorMotorBasic.CLOSE_ANGLE;
                bayDoorState = BayDoorState.CLOSING;
                tolerance = REVERSE_POSITION_TOLERANCE;
                break;
            default:
                throw new IllegalStateException("Unknown Bay Door Action: " + action);
        }

        return run(() -> {
                    moveToPosition(leftMotor, leftMotor.getRotation(), goalPosition, tolerance);
                    moveToPosition(rightMotor, rightMotor.getRotation(), goalPosition, tolerance);
                });
    }

    public Command openBayDoor() {
        return moveBayDoorTo(BayDoorAction.OPEN);
    }

    public Command closeBayDoor() {
        return moveBayDoorTo(BayDoorAction.CLOSE);
    }

    private void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }

    private boolean atHardLeft() {
        return leftHardLimit.get();
    }

    private boolean atHardRight() {
        return rightHardLimit.get();
    }

    private void resetEncoders() {
        leftMotor.resetRelativeEncoder();
        rightMotor.resetRelativeEncoder();
    }

    public Command homeBayDoor() {
        return runEnd(() -> {
            moveTowards(leftMotor, Rotations.of(Integer.MIN_VALUE));
            moveTowards(rightMotor, Rotations.of(Integer.MIN_VALUE));
        }, () -> {
            stop();
            resetEncoders();
            setDefaultCommand(closeBayDoor());
        }).until(() -> atHardLeft() && atHardRight());
    }
}

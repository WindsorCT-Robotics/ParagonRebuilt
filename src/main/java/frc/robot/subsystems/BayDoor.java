package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.CanId;
import frc.robot.hardware.DigitalInputOutput;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorDualMotorBasic;

public class BayDoor extends SubsystemBase {
    private final BayDoorDualMotorBasic bayDoorController;
    private final DigitalInput leftHardLimit;
    private final DigitalInput rightHardLimit;

    private static final Dimensionless ROLLER_SHUTTLE_DUTY_CYCLE = Percent.of(0.1);
    private static final Dimensionless BAY_DOOR_ACTION_DUTY_CYCLE = Percent.of(0.1);
    private static final Dimensionless HOME_BAY_DOOR_DUTY_CYCLE = Percent.of(-0.1);

    private static final IdleMode OPEN_IDLE_MODE = IdleMode.kCoast;
    private static final IdleMode OPEN_INTAKE_IDLE_MODE = IdleMode.kBrake;
    private static final IdleMode CLOSE_IDLE_MODE = IdleMode.kBrake;

    private BayDoorState bayDoorState = BayDoorState.UNKNOWN;

    public BayDoor(
            String name,
            CanId intakeBayDoorLeftMotorCanId,
            CanId intakeBayDoorRightMotorCanId,
            DigitalInputOutput leftLimitSwitchDIO,
            DigitalInputOutput rightLimitSwitchDIO) {
        SendableRegistry.add(this, name);
        this.bayDoorController = new BayDoorDualMotorBasic("Intake Bay Door", intakeBayDoorLeftMotorCanId,
                intakeBayDoorRightMotorCanId);
        leftHardLimit = new DigitalInput(leftLimitSwitchDIO.Id());
        rightHardLimit = new DigitalInput(rightLimitSwitchDIO.Id());
    }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Left Limit", atLeftHardClosed());
        // SmartDashboard.putBoolean("Right Limit", atRightHardClosed());
        // SmartDashboard.putString("Bay Door State", bayDoorState.toString());
        // SmartDashboard.putNumber("Left Motor Encoder Degrees",
        // bayDoorController.getLeftRotation().in(Degrees));
        // SmartDashboard.putNumber("Left Motor Encoder Rotations",
        // bayDoorController.getLeftRotation().in(Rotations));
        // SmartDashboard.putNumber("Right Motor Encoder Degrees",
        // bayDoorController.getRightRotation().in(Degrees));
        // SmartDashboard.putNumber("Right Motor Encoder Rotations",
        // bayDoorController.getRightRotation().in(Rotations));
    }

    private enum BayDoorState {
        UNKNOWN,
        OPENING,
        OPENED,
        CLOSING,
        CLOSED
    }

    private enum BayDoorAction {
        OPEN,
        OPEN_AND_INTAKE,
        CLOSE,
        HOME
    }

    // should home individual motors
    public Command homeBayDoor() {
        return runEnd(() -> positionBayDoorTo(BayDoorAction.HOME), () -> {
            setDefaultCommand(closeBayDoor());
            bayDoorState = BayDoorState.CLOSED;
            bayDoorController.resetRelativeEncoder();
        }).until(() -> (atLeftHardClosed() && atRightHardClosed()));
    }

    // Should use left and right limits to determine if the individual motor should
    // go any further.
    private void moveToPosition(BayDoorAction bayDoorAction) {
        switch (bayDoorAction) {
            case OPEN:
                if (!atLeftOpen()) {
                    bayDoorController.setLeftMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE);
                } else {
                    bayDoorController.stopLeftMotor();
                }

                if (!atRightOpen()) {
                    bayDoorController.setRightMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE);
                } else {
                    bayDoorController.stopRightMotor();
                }

                bayDoorController.setIdleMode(OPEN_IDLE_MODE);
                bayDoorState = BayDoorState.OPENING;
                break;
            case OPEN_AND_INTAKE:
                if (!atLeftOpen()) {
                    bayDoorController.setLeftMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE);
                } else {
                    bayDoorController.stopLeftMotor();
                }

                if (!atRightOpen()) {
                    bayDoorController.setRightMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE);
                } else {
                    bayDoorController.stopRightMotor();
                }

                bayDoorController.setIdleMode(OPEN_INTAKE_IDLE_MODE);
                bayDoorState = BayDoorState.OPENING;
                break;
            case CLOSE:
                if (!atLeftClosed()) {
                    bayDoorController.setLeftMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE.times(-1));
                } else {
                    bayDoorController.stopLeftMotor();
                }

                if (!atRightClosed()) {
                    bayDoorController.setRightMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE.times(-1));
                } else {
                    bayDoorController.stopRightMotor();
                }
                bayDoorController.setIdleMode(CLOSE_IDLE_MODE);
                bayDoorState = BayDoorState.CLOSING;
                break;
            case HOME:
                if (!atLeftHardClosed()) {
                    bayDoorController.setDutyCycle(HOME_BAY_DOOR_DUTY_CYCLE);
                } else {
                    bayDoorController.stopLeftMotor();
                }

                if (!atRightHardClosed()) {
                    bayDoorController.setDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE);
                } else {
                    bayDoorController.stopRightMotor();
                }

                bayDoorController.setIdleMode(CLOSE_IDLE_MODE);
                bayDoorState = BayDoorState.CLOSING;
            default:
                throw new IllegalStateException("Unknown Bay Door Action: " + bayDoorAction);
        }

    }

    // Some how account for the differences in motor rotation. If one motor reaches
    // its final position the other should keep going to the final location
    // regardless of the other motor.
    private Command positionBayDoorTo(BayDoorAction bayDoorAction) {
        return runEnd(() -> moveToPosition(bayDoorAction), () -> bayDoorController.stop());
    }

    public Command openBayDoor() {
        return positionBayDoorTo(BayDoorAction.OPEN);
    }

    public Command closeBayDoor() {
        return positionBayDoorTo(BayDoorAction.CLOSE);
    }

    private Command openBayDoorAndHold() {
        return positionBayDoorTo(BayDoorAction.OPEN_AND_INTAKE);
    }

    public Command wiggleBayDoor() {
        return closeBayDoor().andThen(openBayDoor()).repeatedly();
        // TODO: Possibly too much
        // wiggle. Test.
    }

    public Command openBayDoorAndBrake() {
        return openBayDoorAndHold();
    }

    private String getBayDoorState() {
        return bayDoorState.toString();
    }

    // Check closed state with limit switch and maybe a soft limit. Check opened
    // state with a soft limit such as angle check.
    private boolean atLeftOpen() {
        return bayDoorController.atLeftMotorSoftForwardLimit();
    }

    private boolean atLeftClosed() {
        return bayDoorController.atRightMotorSoftReverseLimit();
    }

    private boolean atLeftHardClosed() {
        return leftHardLimit.get();
    }

    private boolean atRightOpen() {
        return bayDoorController.atRightMotorSoftForwardLimit();
    }

    private boolean atRightClosed() {
        return bayDoorController.atRightMotorSoftReverseLimit();
    }

    private boolean atRightHardClosed() {
        return rightHardLimit.get();
    }
}
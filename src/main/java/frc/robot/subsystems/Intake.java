package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.CanId;
import frc.robot.hardware.DigitalInputOutput;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorDualMotorBasic;
import frc.robot.hardware.intake_motors.IntakeRollerMotor;

public class Intake extends SubsystemBase {
    private final IntakeRollerMotor rollerMotor;
    private final BayDoorDualMotorBasic bayDoorController;
    private BayDoorState bayDoorState = BayDoorState.UNKNOWN;
    private static final Dimensionless ROLLER_INTAKE_DUTY_CYCLE = Percent.of(0.1);
    private static final Dimensionless ROLLER_SHUTTLE_DUTY_CYCLE = Percent.of(0.1);
    private static final Dimensionless BAY_DOOR_ACTION_DUTY_CYCLE = Percent.of(0.1);
    private static final Dimensionless HOME_BAY_DOOR_DUTY_CYCLE = Percent.of(-0.1);
    private static final IdleMode OPEN_IDLE_MODE = IdleMode.kCoast;
    private static final IdleMode OPEN_INTAKE_IDLE_MODE = IdleMode.kBrake;
    private static final IdleMode CLOSE_IDLE_MODE = IdleMode.kBrake;
    private final DigitalInput leftHardLimit;
    private final DigitalInput rightHardLimit;

    public Intake(
            String name,
            CanId intakeRollerMotorCanId,
            CanId intakeBayDoorLeftMotorCanId,
            CanId intakeBayDoorRightMotorCanId,
            DigitalInputOutput leftLimitSwitchDIO,
            DigitalInputOutput rightLimitSwitchDIO) {
        SendableRegistry.add(this, name);
        this.rollerMotor = new IntakeRollerMotor("Intake Roller Motor", intakeRollerMotorCanId);
        this.bayDoorController = new BayDoorDualMotorBasic("Intake Bay Door", intakeBayDoorLeftMotorCanId,
                intakeBayDoorRightMotorCanId);
        leftHardLimit = new DigitalInput(leftLimitSwitchDIO.Id());
        rightHardLimit = new DigitalInput(rightLimitSwitchDIO.Id());
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Left Limit", atLeftHardClosed());
        SmartDashboard.putBoolean("Right Limit", atRightHardClosed());
        SmartDashboard.putString("Bay Door State", bayDoorState.toString());
        SmartDashboard.putNumber("Left Motor Encoder Degrees", bayDoorController.getLeftRotation().in(Degrees));
        SmartDashboard.putNumber("Left Motor Encoder Rotations", bayDoorController.getLeftRotation().in(Rotations));
        SmartDashboard.putNumber("Right Motor Encoder Degrees", bayDoorController.getRightRotation().in(Degrees));
        SmartDashboard.putNumber("Right Motor Encoder Rotations", bayDoorController.getRightRotation().in(Rotations));
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
        CLOSE
    }

    public Command homeBayDoor() {
        return runEnd(() -> bayDoorController.setDutyCycle(HOME_BAY_DOOR_DUTY_CYCLE), () -> {
            bayDoorController.stop();
            setDefaultCommand(closeBayDoor());
            bayDoorState = BayDoorState.CLOSED;
            bayDoorController.resetRelativeEncoder();
        }).until(this::isClosed);
    }

    public Command resetEncoders() {
        return Commands.runOnce(() -> bayDoorController.resetRelativeEncoder());
    }

    public Command moveBayDoor(Supplier<Dimensionless> percent) {
        return runEnd(() -> bayDoorController.setDutyCycle(percent.get()), () -> bayDoorController.stop());
    }

    // With `getRotation()` figure the condition to decide if
    // Shouldn't need to set the duty cycle of both motors with seperate methods.
    // Since both should rotate until final direction it met so they should be
    // individually stop when needed.
    private void moveToPosition(BayDoorAction bayDoorAction) {
        switch (bayDoorAction) {
            case OPEN:
                if (!atSoftForwardLimit()) {
                    bayDoorController.setLeftMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE);
                    bayDoorController.setRightMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE);
                    bayDoorController.setIdleMode(OPEN_IDLE_MODE);
                    bayDoorState = BayDoorState.OPENING;
                } else {
                    bayDoorController.stop();
                }
                break;
            case OPEN_AND_INTAKE:
                if (!atSoftForwardLimit()) {
                    bayDoorController.setLeftMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE);
                    bayDoorController.setRightMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE);
                    bayDoorController.setIdleMode(OPEN_INTAKE_IDLE_MODE);
                    bayDoorState = BayDoorState.OPENING;
                } else {
                    bayDoorController.stop();
                }
                break;
            case CLOSE:
                if (!atSoftReverseLimit()) {
                    bayDoorController.setLeftMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE.times(-1));
                    bayDoorController.setRightMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE.times(-1));
                    bayDoorController.setIdleMode(CLOSE_IDLE_MODE);
                    bayDoorState = BayDoorState.CLOSING;
                    bayDoorState = BayDoorState.OPENING;
                } else {
                    bayDoorController.stop();
                }
                break;
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
        return positionBayDoorTo(BayDoorAction.CLOSE).unless(this::isClosed).until(this::isClosed);
    }

    private Command openBayDoorAndHold() {
        return positionBayDoorTo(BayDoorAction.OPEN_AND_INTAKE);
    }

    // public Command wiggleBayDoor() {
    // return
    // closeBayDoor().andThen(openBayDoor()).alongWith(intakeFuel()).repeatedly();
    // // TODO: Possibly too much
    // // wiggle. Test.
    // }

    public Command intakeFuel(Supplier<Dimensionless> percent) {
        return Commands.runEnd(() -> {
            if (bayDoorState != BayDoorState.CLOSED) {
                rollerMotor.setDutyCycle(percent.get());
            } else {
                rollerMotor.stop();
            }
        }, () -> {
            rollerMotor.stop();
        });
    }

    private Command shuttleFuel() {
        return Commands.run(() -> {
            if (bayDoorState != BayDoorState.CLOSED) {
                rollerMotor.setDutyCycle(ROLLER_SHUTTLE_DUTY_CYCLE);
            } else {
                rollerMotor.stop();
            }
        });
    }

    // public Command openBayDoorAndIntakeFuel() {
    // return openBayDoorAndHold().andThen(intakeFuel());
    // }

    // public Command openBayDoorAndShuttleFuel() {
    // return openBayDoorAndHold().andThen(shuttleFuel());
    // }

    // private String getBayDoorState() {
    // return bayDoorState.toString();
    // }

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

    private boolean isClosed() {
        return atLeftHardClosed() || atRightHardClosed();
    }

    private boolean isOpen() {
        return atLeftOpen() || atRightOpen();
    }

    private boolean atSoftForwardLimit() {
        return bayDoorController.atLeftMotorSoftForwardLimit() || bayDoorController.atRightMotorSoftForwardLimit();
    }

    private boolean atSoftReverseLimit() {
        return bayDoorController.atLeftMotorSoftReverseLimit() || bayDoorController.atRightMotorSoftReverseLimit();
    }
}
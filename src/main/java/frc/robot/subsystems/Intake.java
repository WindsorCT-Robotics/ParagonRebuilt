package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private static final Dimensionless ROLLER_INTAKE_DUTY_CYCLE = Percent.of(1);
    private static final Dimensionless ROLLER_SHUTTLE_DUTY_CYCLE = Percent.of(1);
    private static final Dimensionless BAY_DOOR_ACTION_DUTY_CYCLE = Percent.of(1);
    private static final Dimensionless HOME_BAY_DOOR_DUTY_CYCLE = Percent.of(-1);
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
        return new ParallelCommandGroup(
                Commands.runEnd(
                        () -> bayDoorController.setDutyCycle(HOME_BAY_DOOR_DUTY_CYCLE),
                        () -> setDefaultCommand(closeBayDoor())).until(this::atLeftHardClosed),
                Commands.runEnd(
                        () -> bayDoorController.setDutyCycle(HOME_BAY_DOOR_DUTY_CYCLE),
                        () -> setDefaultCommand(closeBayDoor())).until(this::atRightHardClosed));
    }

    // With `getRotation()` figure the condition to decide if
    // Shouldn't need to set the duty cycle of both motors with seperate methods.
    // Since both should rotate until final direction it met so they should be
    // individually stop when needed.
    private Command moveToPosition(BayDoorAction bayDoorAction) {
        switch (bayDoorAction) {
            case OPEN:
                return new SequentialCommandGroup(
                        Commands.runOnce(() -> {
                            bayDoorController.setIdleMode(OPEN_IDLE_MODE);
                            bayDoorState = BayDoorState.OPENING;
                        }),
                        new ParallelCommandGroup(
                                Commands.runEnd(
                                        () -> bayDoorController.setLeftMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE),
                                        () -> bayDoorController.stopLeftMotor()).unless(this::atLeftOpen),
                                Commands.runEnd(
                                        () -> bayDoorController.setRightMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE),
                                        () -> bayDoorController.stopRightMotor()).until(this::atRightOpen)),
                        Commands.runOnce(() -> bayDoorState = BayDoorState.OPENED));
            case OPEN_AND_INTAKE:
                return new SequentialCommandGroup(
                        Commands.runOnce(() -> {
                            bayDoorController.setIdleMode(OPEN_INTAKE_IDLE_MODE);
                            bayDoorState = BayDoorState.OPENING;
                        }),
                        new ParallelCommandGroup(
                                Commands.runEnd(
                                        () -> bayDoorController.setLeftMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE),
                                        () -> bayDoorController.stopLeftMotor()).unless(this::atLeftOpen),
                                Commands.runEnd(
                                        () -> bayDoorController.setRightMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE),
                                        () -> bayDoorController.stopRightMotor()).until(this::atRightOpen)),
                        Commands.runOnce(() -> bayDoorState = BayDoorState.OPENED));

            case CLOSE:
                return new SequentialCommandGroup(
                        Commands.runOnce(() -> {
                            bayDoorController.setIdleMode(CLOSE_IDLE_MODE);
                            bayDoorState = BayDoorState.CLOSING;
                        }),
                        new ParallelCommandGroup(
                                Commands.runEnd(
                                        () -> bayDoorController
                                                .setLeftMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE.times(-1)),
                                        () -> bayDoorController.stopLeftMotor()).unless(this::atLeftClosed),
                                Commands.runEnd(
                                        () -> bayDoorController
                                                .setRightMotorDutyCycle(BAY_DOOR_ACTION_DUTY_CYCLE.times(-1)),
                                        () -> bayDoorController.stopRightMotor()).until(this::atRightClosed)),
                        Commands.runOnce(() -> bayDoorState = BayDoorState.CLOSED));
            default:
                throw new IllegalStateException("Unknown Bay Door Action: " + bayDoorAction);
        }
    }

    // Some how account for the differences in motor rotation. If one motor reaches
    // its final position the other should keep going to the final location
    // regardless of the other motor.
    private Command positionBayDoorTo(BayDoorAction bayDoorAction) {
        return moveToPosition(bayDoorAction);
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

    // public Command wiggleBayDoor() {
    // return
    // closeBayDoor().andThen(openBayDoor()).alongWith(intakeFuel()).repeatedly();
    // // TODO: Possibly too much
    // // wiggle. Test.
    // }

    public Command intakeFuel() {
        return Commands.runEnd(() -> {
            if (bayDoorState != BayDoorState.CLOSED) {
                rollerMotor.setDutyCycle(ROLLER_INTAKE_DUTY_CYCLE);
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
}
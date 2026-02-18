package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.CanId;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorDualMotorBasic;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorMotorBasic;
import frc.robot.hardware.intake_motors.IntakeRollerMotor;

public class Intake extends SubsystemBase {
    private final IntakeRollerMotor rollerMotor;
    private final BayDoorDualMotorBasic bayDoorController;
    private BayDoorState bayDoorState = BayDoorState.UNKNOWN;
    private static final Dimensionless ROLLER_INTAKE_DUTY_CYCLE = Percent.of(1);
    private static final Dimensionless ROLLER_SHUTTLE_DUTY_CYCLE = Percent.of(1);
    private static final Dimensionless HOME_BAY_DOOR_DUTY_CYCLE = Percent.of(-1);
    private static final IdleMode OPEN_IDLE_MODE = IdleMode.kCoast;
    private static final IdleMode OPEN_INTAKE_IDLE_MODE = IdleMode.kBrake;
    private static final IdleMode CLOSE_IDLE_MODE = IdleMode.kBrake;

    public Intake(
            String name,
            CanId intakeRollerMotorCanId,
            CanId intakeBayDoorLeftMotorCanId,
            CanId intakeBayDoorRightMotorCanId) {
        SendableRegistry.add(this, name);
        this.rollerMotor = new IntakeRollerMotor("Intake Roller Motor", intakeRollerMotorCanId);
        this.bayDoorController = new BayDoorDualMotorBasic("Intake Bay Door", intakeBayDoorLeftMotorCanId,
                intakeBayDoorRightMotorCanId);
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

    // public Command homeBayDoor() {
    // return Commands.runEnd(
    // () -> bayDoorController.setDutyCycle(HOME_BAY_DOOR_DUTY_CYCLE),
    // () -> setDefaultCommand(closeBayDoor())).until(isClosed());
    // }

    // With `getRotation()` figure the condition to decide if
    private void moveToPosition(BayDoorAction bayDoorAction) {
        switch (bayDoorAction) {
            case OPEN:
                bayDoorController.setDutyCycle(Percent.of(3)); // TODO: Replace value.
                bayDoorController.setIdleMode(OPEN_IDLE_MODE);
                bayDoorState = BayDoorState.OPENING;
                break;
            case OPEN_AND_INTAKE:
                bayDoorController.setDutyCycle(Percent.of(3)); // TODO: Replace value.
                bayDoorController.setIdleMode(OPEN_INTAKE_IDLE_MODE);
                bayDoorState = BayDoorState.OPENING;
                break;
            case CLOSE:
                bayDoorController.setDutyCycle(Percent.of(-3)); // TODO: Replace value.
                bayDoorController.setIdleMode(CLOSE_IDLE_MODE);
                bayDoorState = BayDoorState.CLOSING;
                break;
        }
    }

    // Some how account for the differences in motor rotation. If one motor reaches
    // its final position the other should keep going to the final location
    // regardless of the other motor.
    private Command positionBayDoorTo(BayDoorAction bayDoorAction) {
        switch (bayDoorAction) {
            case OPEN:
                return Commands.runEnd(
                        () -> moveToPosition(bayDoorAction),
                        () -> bayDoorState = BayDoorState.OPENED)
                        .until(isOpen());
            case CLOSE:
                return Commands.runEnd(
                        () -> moveToPosition(bayDoorAction),
                        () -> bayDoorState = BayDoorState.CLOSED)
                        .until(isClosed());
            default:
                throw new IllegalStateException("Unknown Bay Door Action: " + bayDoorAction);
        }
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
    private boolean isLeftOpen() {

    }

    private boolean isLeftClosed() {

    }

    private boolean isRightOpen() {

    }

    private boolean isRightClosed() {

    }

    private boolean isOpen() {
        return isLeftOpen() && isRightOpen();
    }

    private boolean isClosed() {
        return isLeftClosed() && isRightClosed();
    }
}
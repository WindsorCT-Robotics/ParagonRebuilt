package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hardware.CanId;
import frc.robot.hardware.intakeMotors.IntakeBayDoorDualMotors;
import frc.robot.hardware.intakeMotors.IntakeBayDoorMotor;
import frc.robot.hardware.intakeMotors.IntakeRollerMotor;

public class Intake extends SubsystemBase {
    private final IntakeBayDoorDualMotors bayDoorController;
    private final IntakeRollerMotor rollerMotor;
    private static final Dimensionless ROLLER_INTAKE_DUTY_CYCLE = Percent.of(0); // TODO: Find a good percentage and
                                                                                 // ensure that positive duty cycle
                                                                                 // intakes.
    private static final Dimensionless ROLLER_SHUTTLE_DUTY_CYCLE = Percent.of(0); // TODO: Find a good percentage and
                                                                                  // ensure that negative duty cycle
                                                                                  // shuttles.
    private static final Dimensionless HOME_BAY_DOOR_DUTY_CYCLE = Percent.of(0); // TODO: Determine percentage.
    private static final boolean DUAL_MOTORS_INVERTED = false; // TODO: Which way goes forward?
    private static final IdleMode OPEN_IDLE_MODE = IdleMode.kCoast;
    private static final IdleMode OPEN_INTAKE_IDLE_MODE = IdleMode.kBrake;
    private static final IdleMode CLOSE_IDLE_MODE = IdleMode.kBrake;
    private BayDoorState bayDoorState = BayDoorState.UNKNOWN;

    public Intake(
            String name,
            CanId rollerMotorCanId,
            CanId leadMotorCanId,
            CanId followerMotorCanId) {
        SendableRegistry.add(this, name);
        rollerMotor = new IntakeRollerMotor("Intake Roller Motor", rollerMotorCanId);
        IntakeBayDoorMotor leadMotor = new IntakeBayDoorMotor("Intake Bay Door Motor Lead", leadMotorCanId);
        IntakeBayDoorMotor followerMotor = new IntakeBayDoorMotor("Intake Bay Door Motor Follower", followerMotorCanId);
        bayDoorController = new IntakeBayDoorDualMotors("Intake Bay Door Dual Motors", leadMotor, followerMotor,
                DUAL_MOTORS_INVERTED);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addStringProperty("Intake Bay Door State: ", this::getBayDoorState, null);
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

    /*
     * IGNORE THOUGHT DUMP.
     * Position Bay Door based on an enum.
     * 
     * When the command is runned it should check if the limit switch is hit or not,
     * although RevLib already does that with its limit switch behavior.
     * 
     * Be able to simply run the rollers.
     * 
     * Create a sequencial command of opening and roller fuel.
     * 
     * Should be a state of baydoor open but not rollers moving and bayDoor motors
     * should be on coast.
     * 
     * Moves bayDoor to reverse limit switch, once triggered it stops and sets the
     * `BayDoorState`
     * 
     * There should be 2 different types of OPENED. One where the intake is actively
     * on where the idle mode should be kBrake and if that's not enough maybe a
     * small duty cycle. The other one is when the rollers are actively on. This is
     * to reduce possible intake damage if a potential robot hits into us and it was
     * also initally the entire reason we picked this design.
     * 
     * Should the the bay door be 2 separate mechanism? Does the rollers interfere
     * with anything when rolling while closed? The build team said that potentially
     * to get the remaining balls that're stuck in the intake.
     */

    public Command homeBayDoor() {
        return Commands.runEnd(
                () -> bayDoorController.setDutyCycle(HOME_BAY_DOOR_DUTY_CYCLE),
                () -> bayDoorState = BayDoorState.CLOSED).until(isClosed());
    }

    private void setPositionBayDoorTo(BayDoorAction bayDoorAction) {
        switch (bayDoorAction) {
            case OPEN:
                bayDoorController.setAngularPosition(IntakeBayDoorMotor.OPENED_ANGLE);
                bayDoorController.setIdleMode(OPEN_IDLE_MODE);
                bayDoorState = BayDoorState.OPENING;
                break;
            case OPEN_AND_INTAKE:
                bayDoorController.setAngularPosition(IntakeBayDoorMotor.OPENED_ANGLE);
                bayDoorController.setIdleMode(OPEN_INTAKE_IDLE_MODE);
                bayDoorState = BayDoorState.OPENING;
                break;
            case CLOSE:
                bayDoorController.setAngularPosition(IntakeBayDoorMotor.CLOSED_ANGLE);
                bayDoorController.setIdleMode(CLOSE_IDLE_MODE);
                bayDoorState = BayDoorState.CLOSING;
                break;
        }
    }

    private Command positionBayDoorTo(BayDoorAction bayDoorAction) {
        switch (bayDoorAction) {
            case OPEN:
                return Commands.runEnd(
                        () -> setPositionBayDoorTo(bayDoorAction),
                        () -> bayDoorState = BayDoorState.OPENED)
                        .until(isOpen());
            case CLOSE:
                return Commands.runEnd(
                        () -> setPositionBayDoorTo(bayDoorAction),
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

    public Command wiggleBayDoor() {
        return closeBayDoor().andThen(openBayDoor()).repeatedly(); // TODO: Possibly too much wiggle. Test.
    }

    private Command intakeFuel() {
        return Commands.run(() -> {
            if (bayDoorState != BayDoorState.CLOSED) {
                rollerMotor.setDutyCycle(ROLLER_INTAKE_DUTY_CYCLE);
            } else {
                rollerMotor.stop();
            }
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

    // This command should be able to deploy the bay door and start intaking fuel
    // once toggled. When this command ends it should call
    // `positionBayDoorTo(BayDoorAction.OPEN)` to let loose unless it starts
    // dragging on the floor then consider removing BayDoorAction.OPEN_AND_INTAKE
    // and have the line say:
    // `positionBayDoorTo(BayDoorAction.OPEN).andThen(intakeFuel())`.
    public Command openBayDoorAndIntakeFuel() {
        return openBayDoorAndHold().andThen(intakeFuel());
    }

    public Command openBayDoorAndShuttleFuel() {
        return openBayDoorAndHold().andThen(shuttleFuel());
    }

    private String getBayDoorState() {
        return bayDoorState.toString();
    }

    private final Trigger isClosed() {
        return bayDoorController.isClosed();
    }

    private final Trigger isOpen() {
        return bayDoorController.isOpen();
    }
}

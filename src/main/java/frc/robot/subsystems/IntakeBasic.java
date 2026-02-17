package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hardware.CanId;
import frc.robot.hardware.intakeMotors.IntakeBayDoorDualMotorsBasic;
import frc.robot.hardware.intakeMotors.IntakeBayDoorMotor;
import frc.robot.hardware.intakeMotors.IntakeBayDoorMotorBasic;
import frc.robot.hardware.intakeMotors.IntakeRollerMotor;

public class IntakeBasic extends SubsystemBase {
    private final IntakeBayDoorDualMotorsBasic bayDoorController;
    private final IntakeRollerMotor rollerMotor;
    private static final Dimensionless ROLLER_INTAKE_DUTY_CYCLE = Percent.of(10); // TODO: Find a good percentage and
                                                                                  // ensure that positive duty cycle
                                                                                  // intakes.
    private static final Dimensionless ROLLER_SHUTTLE_DUTY_CYCLE = Percent.of(-10); // TODO: Find a good percentage and
                                                                                    // ensure that negative duty cycle
                                                                                    // shuttles.
    private static final Dimensionless HOME_BAY_DOOR_DUTY_CYCLE = Percent.of(-5); // TODO: Determine percentage.
    private static final boolean DUAL_MOTORS_INVERTED = false; // TODO: Which way goes forward?
    private static final IdleMode OPEN_IDLE_MODE = IdleMode.kCoast;
    private static final IdleMode OPEN_INTAKE_IDLE_MODE = IdleMode.kBrake;
    private static final IdleMode CLOSE_IDLE_MODE = IdleMode.kBrake;
    private BayDoorState bayDoorState = BayDoorState.UNKNOWN;

    public IntakeBasic(
            String name,
            CanId rollerMotorCanId,
            CanId leadMotorCanId,
            CanId followerMotorCanId) {
        SendableRegistry.add(this, name);
        rollerMotor = new IntakeRollerMotor("Intake Roller Motor", rollerMotorCanId);
        IntakeBayDoorMotorBasic leadMotor = new IntakeBayDoorMotorBasic("Intake Bay Door Motor Lead", leadMotorCanId);
        IntakeBayDoorMotorBasic followerMotor = new IntakeBayDoorMotorBasic("Intake Bay Door Motor Follower",
                followerMotorCanId);
        bayDoorController = new IntakeBayDoorDualMotorsBasic("Intake Bay Door Dual Motors", leadMotor, followerMotor,
                DUAL_MOTORS_INVERTED);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addStringProperty("Intake Bay Door State: ", this::getBayDoorState, null);
        builder.addDoubleProperty("Voltage (V)", this::getVoltage, this::setVoltage);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LIMIT", bayDoorController.isClosed().getAsBoolean());
    }

    private double getVoltage() {
        return bayDoorController.getVoltage().in(Volts);
    }

    private void setVoltage(double voltage) {
        bayDoorController.setVoltage(Volts.of(voltage));
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
        return Commands.runEnd(
                () -> bayDoorController.setDutyCycle(HOME_BAY_DOOR_DUTY_CYCLE),
                () -> setDefaultCommand(closeBayDoor()), this).until(isClosed());
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
            case OPEN, OPEN_AND_INTAKE:
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
        return closeBayDoor().andThen(openBayDoor()).alongWith(intakeFuel()).repeatedly(); // TODO: Possibly too much
                                                                                           // wiggle. Test.
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

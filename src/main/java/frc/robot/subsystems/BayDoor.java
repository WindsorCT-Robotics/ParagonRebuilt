package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.CanId;
import frc.robot.hardware.DigitalInputOutput;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorDualMotorBasic;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorMotorBasic;

public class BayDoor extends SubsystemBase {
    private final BayDoorDualMotorBasic bayDoorController;
    private final DigitalInput leftHardLimit;
    private final DigitalInput rightHardLimit;

    private static final Dimensionless ROLLER_SHUTTLE_DUTY_CYCLE = Percent.of(0.1);
    private static final Dimensionless BAY_DOOR_ACTION_DUTY_CYCLE = Percent.of(0.1);
    private static final Dimensionless HOME_BAY_DOOR_DUTY_CYCLE = Percent.of(-0.1);
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.01); // change values if needed.
    private static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(0.01);
    private static final TrapezoidProfile.Constraints MOTION_PROFILE_CONSTRAINTS = new Constraints(
            MAX_ANGULAR_VELOCITY.in(RotationsPerSecond), MAX_ANGULAR_ACCELERATION.in(RotationsPerSecondPerSecond));

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
        SmartDashboard.putBoolean("Left Limit", atLeftHardClosed());
        SmartDashboard.putBoolean("Right Limit", atRightHardClosed());
        SmartDashboard.putString("Bay Door State", bayDoorState.toString());
        SmartDashboard.putNumber("Left Motor Encoder Degrees",
        bayDoorController.getLeftRotation().in(Degrees));
        SmartDashboard.putNumber("Left Motor Encoder Rotations",
        bayDoorController.getLeftRotation().in(Rotations));
        SmartDashboard.putNumber("Right Motor Encoder Degrees",
        bayDoorController.getRightRotation().in(Degrees));
        SmartDashboard.putNumber("Right Motor Encoder Rotations",
        bayDoorController.getRightRotation().in(Rotations));
        SmartDashboard.putNumber("Left Duty Cycle", bayDoorController.getLeftMotorDutyCycle().in(Percent));
        SmartDashboard.putNumber("Right Duty Cycle", bayDoorController.getRightMotorDutyCycle().in(Percent));
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
        System.out.println("KKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK");
        return runEnd(() -> positionBayDoorTo(BayDoorAction.HOME), () -> {
            setDefaultCommand(closeBayDoor());
            bayDoorState = BayDoorState.CLOSED;
            bayDoorController.resetRelativeEncoder();
        }).until(() -> (atLeftHardClosed() && atRightHardClosed()));
    }

    // Should use left and right limits to determine if the individual motor should
    // go any further.
    private void moveToPosition(BayDoorAction bayDoorAction) {
        // System.out.println("TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT");
        SmartDashboard.putString("BayDoor Action", bayDoorAction.toString());
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

    private AngularVelocity calculateAngularVelocity(Angle goalPosition, Angle currentPosition,
            AngularVelocity currentVelocity) {
        TrapezoidProfile.State currentState = new State(currentPosition.in(Rotations),
                currentVelocity.in(RotationsPerSecond));

        TrapezoidProfile.State goalState = new State(goalPosition.in(Rotations),
                RotationsPerSecond.zero().in(RotationsPerSecond));

        TrapezoidProfile profile = new TrapezoidProfile(MOTION_PROFILE_CONSTRAINTS);

        return RotationsPerSecond.of(profile.calculate(TimedRobot.kDefaultPeriod, currentState, goalState).velocity);
    }

    private void motionProfileLeftMotor(Angle goalPosition) {
        bayDoorController.setLeftMotorRPS(
                calculateAngularVelocity(
                        goalPosition,
                        bayDoorController.getLeftRotation(),
                        MAX_ANGULAR_VELOCITY));
    }

    private void motionProfileRightMotor(Angle goalPosition) {
        bayDoorController.setRightMotorRPS(
                calculateAngularVelocity(
                        goalPosition,
                        bayDoorController.getRightRotation(),
                        MAX_ANGULAR_VELOCITY));
    }

    private void motionProfileBayDoorTo(BayDoorAction bayDoorAction) {
        switch (bayDoorAction) {
            case OPEN:
                if (!atLeftOpen()) {
                    motionProfileLeftMotor(BayDoorMotorBasic.OPEN_ANGLE);
                } else {
                    bayDoorController.stopLeftMotor();
                }

                if (!atRightOpen()) {
                    motionProfileRightMotor(BayDoorMotorBasic.OPEN_ANGLE);
                } else {
                    bayDoorController.stopRightMotor();
                }

                bayDoorController.setIdleMode(OPEN_IDLE_MODE);
                bayDoorState = BayDoorState.OPENING;
                break;
            case OPEN_AND_INTAKE:
                if (!atLeftOpen()) {
                    motionProfileLeftMotor(BayDoorMotorBasic.OPEN_ANGLE);
                } else {
                    bayDoorController.stopLeftMotor();
                }

                if (!atRightOpen()) {
                    motionProfileRightMotor(BayDoorMotorBasic.OPEN_ANGLE);
                } else {
                    bayDoorController.stopRightMotor();
                }

                bayDoorController.setIdleMode(OPEN_INTAKE_IDLE_MODE);
                bayDoorState = BayDoorState.OPENING;
                break;
            case CLOSE:
                if (!atLeftClosed()) {
                    motionProfileLeftMotor(BayDoorMotorBasic.CLOSE_ANGLE);
                } else {
                    bayDoorController.stopLeftMotor();
                }

                if (!atRightClosed()) {
                    motionProfileRightMotor(BayDoorMotorBasic.CLOSE_ANGLE);
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
        return runEnd(() -> moveToPosition(bayDoorAction), () -> bayDoorController.stop())
                .handleInterrupt(() -> {
                    bayDoorState = BayDoorState.UNKNOWN;
                    setDefaultCommand(homeBayDoor());
                }
                );
        // TODO: See if motion profiling works.
    } // TODO: remove the dual motor class.

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
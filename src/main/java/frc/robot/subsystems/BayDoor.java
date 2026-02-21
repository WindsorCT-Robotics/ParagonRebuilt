package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.CanId;
import frc.robot.hardware.DigitalInputOutput;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorMotorBasic;
import frc.robot.hardware.basic_implementations.intake_motors.BayDoorState;

public class BayDoor extends SubsystemBase {
    private final BayDoorMotorBasic leftMotor;
    private final BayDoorMotorBasic rightMotor;
    private final DigitalInput leftHardLimit;
    private final DigitalInput rightHardLimit;
    private static final Angle FORWARD_POSITION_TOLERANCE = Rotations.of(1);
    private static final Angle REVERSE_POSITION_TOLERANCE = Rotations.of(1);
    private static final Dimensionless DEFAULT_DUTY_CYCLE = Percent.of(0.1);
    private static final Dimensionless HOME_DUTY_CYCLE = Percent.of(-0.1);
    private static final boolean INVERTED = true;
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(1);
    private static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(1);
    private static final TrapezoidProfile.Constraints MOTION_CONSTRAINTS = new Constraints(
            MAX_ANGULAR_VELOCITY.in(RotationsPerSecond), MAX_ANGULAR_ACCELERATION.in(RotationsPerSecondPerSecond));

    public BayDoor(
            String name,
            CanId leftMotorId,
            CanId rightMotorId,
            DigitalInputOutput leftLimitSwitchDIO,
            DigitalInputOutput rightLimitSwitchDIO) {
        leftMotor = new BayDoorMotorBasic("Left Motor", leftMotorId);
        rightMotor = new BayDoorMotorBasic("Right Motor", rightMotorId);
        leftHardLimit = new DigitalInput(leftLimitSwitchDIO.Id());
        rightHardLimit = new DigitalInput(rightLimitSwitchDIO.Id());

        // TODO: Be able to apply configuration and keep the same ResetMode and
        // PersisMode without hard coding.
        leftMotor.configure(motor -> {
            motor.configure(new SparkMaxConfig().inverted(INVERTED), ResetMode.kNoResetSafeParameters,
                    PersistMode.kPersistParameters);
        });

        rightMotor.configure(motor -> {
            motor.configure(new SparkMaxConfig().inverted(!INVERTED), ResetMode.kNoResetSafeParameters,
                    PersistMode.kPersistParameters);
        });
        SendableRegistry.add(leftHardLimit, name, name);
        SendableRegistry.addChild(this, leftMotor);
        SendableRegistry.addChild(this, rightMotor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    private enum BayDoorAction {
        OPEN,
        CLOSE
    }

    private void moveTowards(
            BayDoorMotorBasic motor,
            Angle position,
            TrapezoidProfile.Constraints constraints) {
        TrapezoidProfile.State currentState = new State(motor.getAngle().in(Rotations),
                motor.getVelocity().in(RotationsPerSecond));
        TrapezoidProfile.State goalState = new State(position.in(Rotations),
                RotationsPerSecond.zero().in(RotationsPerSecond));
        TrapezoidProfile motionProfile = new TrapezoidProfile(constraints);

        AngularVelocity velocity = RotationsPerSecond
                .of(motionProfile.calculate(TimedRobot.kDefaultPeriod, currentState, goalState).velocity);

        motor.setRPS(velocity);
        SmartDashboard.putNumber("Desired Position", goalState.position);
        SmartDashboard.putNumber("Velocity", velocity.in(RotationsPerSecond));
    }

    private void moveToPosition(
            BayDoorMotorBasic motor,
            Angle position,
            Angle goalPosition,
            Angle tolerance,
            BayDoorState medianState,
            BayDoorState endState) {
        if (!position.isNear(goalPosition, tolerance)) {
            moveTowards(motor, goalPosition, MOTION_CONSTRAINTS);
            motor.setBayMotorState(medianState);
        } else {
            motor.stop();
            motor.setBayMotorState(endState);
        }
    }

    private Command moveBayDoorTo(BayDoorAction action) {
        Angle goalPosition;
        Angle tolerance;
        BayDoorState medianState;
        BayDoorState endState;

        switch (action) {
            case OPEN:
                goalPosition = BayDoorMotorBasic.OPEN_ANGLE;
                leftMotor.setBayMotorState(BayDoorState.OPENING);
                rightMotor.setBayMotorState(BayDoorState.OPENING);
                tolerance = FORWARD_POSITION_TOLERANCE;
                medianState = BayDoorState.OPENING;
                endState = BayDoorState.OPEN;
                break;
            case CLOSE:
                goalPosition = BayDoorMotorBasic.CLOSE_ANGLE;
                leftMotor.setBayMotorState(BayDoorState.CLOSING);
                rightMotor.setBayMotorState(BayDoorState.CLOSING);
                tolerance = REVERSE_POSITION_TOLERANCE;
                medianState = BayDoorState.CLOSING;
                endState = BayDoorState.CLOSE;
                break;
            default:
                throw new IllegalStateException("Unknown Bay Door Action: " + action);
        }

        return run(() -> {
            moveToPosition(leftMotor, leftMotor.getAngle(), goalPosition, tolerance, medianState, endState);
            moveToPosition(rightMotor, rightMotor.getAngle(), goalPosition, tolerance, medianState, endState);
        });
    }

    public Command openBayDoor() {
        return moveBayDoorTo(BayDoorAction.OPEN);
    }

    public Command closeBayDoor() {
        return moveBayDoorTo(BayDoorAction.CLOSE).andThen(homeBayDoor());
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
            leftMotor.setDutyCycle(HOME_DUTY_CYCLE);
            rightMotor.setDutyCycle(HOME_DUTY_CYCLE);
        }, () -> {
            stop();
            resetEncoders();
            setDefaultCommand(closeBayDoor());
        }).until(() -> atHardLeft() && atHardRight());
    }
}

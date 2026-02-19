// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.hardware.CanId;
import frc.robot.hardware.DigitalInputOutput;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Drive.RelativeReference;

public class RobotContainer implements Sendable {
  private static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
  private final Telemetry logger;

  private final Drive drive;
  private final Intake intake;
  private final Spindexer spindexer;

  private static final CanId INTAKE_ROLLER_MOTOR_CAN_ID = new CanId((byte) 16);
  private static final CanId INTAKE_LEFT_BAYDOOR_MOTOR_CAN_ID = new CanId((byte) 14);
  private static final CanId INTAKE_RIGHT_BAYDOOR_MOTOR_CAN_ID = new CanId((byte) 15);
  private static final DigitalInputOutput INTAKE_LEFT_BAYDOOR_DIO = new DigitalInputOutput((byte) 1);
  private static final DigitalInputOutput INTAKE_RIGHT_BAYDOOR_DIO = new DigitalInputOutput((byte) 0);

  private static final CanId SPINDEXER_MOTOR_CAN_ID = new CanId((byte) 13);

  private final CommandXboxController controller;
  private final CommandXboxController operator;
  private static final double MOVE_ROBOT_CURVE = 3.0;
  private static final double TURN_ROBOT_CURVE = 2.0;
  private RelativeReference relativeReference;

  private final SendableChooser<Command> autonomousChooser;
  private static final String DEFAULT_AUTO = ""; // TODO: Once formed autos pick an auto to default to.

  public RobotContainer() {
    try {
      drive = new Drive(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);
    } catch (IOException | ParseException e) {
      throw new IllegalStateException("PathPlanner Configuration failed to load.", e);
    }

    intake = new Intake(
        "Intake",
        INTAKE_ROLLER_MOTOR_CAN_ID,
        INTAKE_LEFT_BAYDOOR_MOTOR_CAN_ID,
        INTAKE_RIGHT_BAYDOOR_MOTOR_CAN_ID,
        INTAKE_LEFT_BAYDOOR_DIO,
        INTAKE_RIGHT_BAYDOOR_DIO);
    spindexer = new Spindexer("Spindexer", SPINDEXER_MOTOR_CAN_ID);

    relativeReference = RelativeReference.FIELD_CENTRIC;

    controller = new CommandXboxController(0);
    operator = new CommandXboxController(1);

    autonomousChooser = AutoBuilder.buildAutoChooser(DEFAULT_AUTO);
    SmartDashboard.putData("Autonomous", autonomousChooser);
    SmartDashboard.putString("Relative Reference", getRelativeReference().toString());
    configureControllerBindings();

    logger = new Telemetry(MAX_SPEED.in(MetersPerSecond));
    drive.registerTelemetry(logger::telemeterize);
  }

  @Override
  public void initSendable(SendableBuilder builder) {

  }

  private Supplier<Dimensionless> curveAxis(Supplier<Dimensionless> percent, double exponent) {
    return () -> Percent
        .of(Math.abs(Math.pow(percent.get().in(Percent), exponent - 1)) * percent.get().times(-1).in(Percent) * 100);
  }

  private RelativeReference getRelativeReference() {
    return relativeReference;
  }

  private void configureControllerBindings() {
    Supplier<Dimensionless> controllerLeftAxisX = () -> Percent.of(controller.getLeftX());
    Supplier<Dimensionless> controllerLeftAxisY = () -> Percent.of(controller.getLeftY());
    Supplier<Dimensionless> controllerRightAxisX = () -> Percent.of(controller.getRightX());

    drive.setDefaultCommand(drive.moveWithPercentages(
        curveAxis(controllerLeftAxisX, MOVE_ROBOT_CURVE),
        curveAxis(controllerLeftAxisY, MOVE_ROBOT_CURVE),
        curveAxis(controllerRightAxisX, TURN_ROBOT_CURVE),
        this::getRelativeReference));

    intake.setDefaultCommand(intake.homeBayDoor());

    // Switches RelativeReference
    controller.leftBumper().onTrue(Commands.runOnce(() -> {
      if (getRelativeReference() == RelativeReference.ROBOT_CENTRIC) {
        relativeReference = RelativeReference.FIELD_CENTRIC;
      } else {
        relativeReference = RelativeReference.ROBOT_CENTRIC;
      }

      SmartDashboard.putString("Relative Reference", getRelativeReference().toString());
    }));

    // Half Speed
    controller.rightBumper().whileTrue(
        drive.moveWithPercentages(
            curveAxis(() -> controllerLeftAxisX.get().div(2), MOVE_ROBOT_CURVE),
            curveAxis(() -> controllerLeftAxisY.get().div(2), MOVE_ROBOT_CURVE),
            curveAxis(controllerRightAxisX, TURN_ROBOT_CURVE),
            this::getRelativeReference));

    controller.a().toggleOnTrue(
        drive.angleToOutpost(
            curveAxis(controllerLeftAxisX, MOVE_ROBOT_CURVE),
            curveAxis(controllerLeftAxisY, MOVE_ROBOT_CURVE)));

    controller.povDown().onTrue(drive.resetGyro());

    /*
     * Note that each routine should be run exactly once in a single log.
     * TODO: After PID Tuning with sysIdDynamics these are no longer needed until
     * tuning PID again.
     */
    controller.back().and(controller.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
    controller.back().and(controller.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
    controller.start().and(controller.y()).whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    controller.start().and(controller.x()).whileTrue(drive.sysIdQuasistatic(Direction.kReverse));

    // Operator
    operator.a().whileTrue(spindexer.indexFuel());
    operator.b().whileTrue(intake.intakeFuel(() -> Percent.of(operator.getRightY())).asProxy());
    operator.y().whileTrue(intake.moveBayDoor(() -> Percent.of(operator.getRightY() / 20)));
    operator.x().onTrue(intake.resetEncoders());

    operator.povUp().toggleOnTrue(intake.openBayDoor());
    operator.povDown().toggleOnTrue(intake.closeBayDoor());
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}
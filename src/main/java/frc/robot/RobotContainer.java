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
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.RelativeReference;

public class RobotContainer implements Sendable {
  private static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
  private final Telemetry logger;

  private final Drive drive;

  private final CommandXboxController controller;
  private static final double MOVE_ROBOT_CURVE = 2.0;
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

    relativeReference = RelativeReference.FIELD_CENTRIC;

    controller = new CommandXboxController(0);

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
    return () -> Percent.of(Math.pow(percent.get().in(Percent), exponent));
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

    /*
     * Note that each routine should be run exactly once in a single log.
     * TODO: After PID Tuning with sysIdDynamics these are no longer needed until
     * tuning PID again.
     */
    controller.back().and(controller.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
    controller.back().and(controller.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
    controller.start().and(controller.y()).whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    controller.start().and(controller.x()).whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}
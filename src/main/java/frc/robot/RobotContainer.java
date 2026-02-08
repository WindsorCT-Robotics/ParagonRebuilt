// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Percent;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.result.Result;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.CommandError;
import frc.robot.subsystems.Drive.RelativeReference;

public class RobotContainer implements Sendable {
  private final Drive drive;

  private final CommandXboxController controller;
  private static final double moveRootCurve = 2.0;
  private static final double turnRootCurve = 2.0;
  private Supplier<RelativeReference> relativeReference;

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

    relativeReference = () -> RelativeReference.FIELD_CENTRIC;

    controller = new CommandXboxController(0);

    autonomousChooser = AutoBuilder.buildAutoChooser(DEFAULT_AUTO);
    SmartDashboard.putData("Autonomous", autonomousChooser);
    SmartDashboard.putString("Relative Reference", relativeReference.get().name());
    configureControllerBindings();
  }

  @Override
  public void initSendable(SendableBuilder builder) {

  }

  private Dimensionless rootPercent(Supplier<Dimensionless> percent, double root) {
    return Percent.of(Math.pow(percent.get().in(Percent), 1.0 / root));
  }

  private Command angleToOutpost(Result<Command, CommandError> commandState) {
    if (commandState.isFailure()) {
      return Commands.none(); // TODO: Instead of returning a none command it should just cancel the command.
    }

    return commandState.getValue();
  }

  private void configureControllerBindings() {
    drive.setDefaultCommand(drive.moveWithPercentages(
        () -> rootPercent(() -> Percent.of(controller.getLeftX()), moveRootCurve),
        () -> rootPercent(() -> Percent.of(controller.getLeftY()), moveRootCurve),
        () -> rootPercent(() -> Percent.of(controller.getRightX()), turnRootCurve),
        relativeReference));

    // Switches RelativeReference
    // TODO: Allow Drive Subsystem to dynamically switch between relativeReferences.
    controller.leftBumper().onTrue(Commands.runOnce(() -> {
      if (relativeReference.get() == RelativeReference.ROBOT_CENTRIC) {
        relativeReference = () -> RelativeReference.FIELD_CENTRIC;
      } else {
        relativeReference = () -> RelativeReference.ROBOT_CENTRIC;
      }

      SmartDashboard.putString("Relative Reference", relativeReference.get().toString());
    }));

    // Half Speed
    controller.rightBumper().whileTrue(
        drive.moveWithPercentages(
            () -> rootPercent(() -> Percent.of(controller.getLeftX()).div(2), moveRootCurve),
            () -> rootPercent(() -> Percent.of(controller.getLeftY()).div(2), moveRootCurve),
            () -> rootPercent(() -> Percent.of(controller.getRightX()), turnRootCurve),
            relativeReference));

    // TODO: Angle To Outpost should either schedule the command or does nothing and
    // let the default command of drive take over.
    controller.a().toggleOnTrue(angleToOutpost(
        drive.angleToOutpost(
            () -> rootPercent(() -> Percent.of(controller.getLeftX()), moveRootCurve),
            () -> rootPercent(() -> Percent.of(controller.getLeftY()), moveRootCurve))));

    // Note that each routine should be run exactly once in a single log.
    controller.back().and(controller.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
    controller.back().and(controller.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
    controller.start().and(controller.y()).whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    controller.start().and(controller.x()).whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }
}
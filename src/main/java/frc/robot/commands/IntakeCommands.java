package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
  private IntakeCommands() {}

  public static Command intake(Intake intake, AngularVelocity velocity) {
    return Commands.runEnd(() -> intake.setVelocity(velocity), () -> intake.stop(), intake);
  }
}

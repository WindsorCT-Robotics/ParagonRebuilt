package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.spindexer.Spindexer;

public class SpindexerCommands {
  private SpindexerCommands() {}

  public static Command placeHolder(Spindexer spindexer) {
    return Commands.runEnd(null, null, null);
  }
}

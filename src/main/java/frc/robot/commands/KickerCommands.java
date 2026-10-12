package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.kicker.Kicker;

public class KickerCommands {
  private KickerCommands() {}

  public static Command placeHolder(Kicker kicker) {
    return Commands.runEnd(null, null, null);
  }
}

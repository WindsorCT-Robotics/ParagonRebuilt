package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.baydoor.Baydoor;

public class BaydoorCommands {
  private BaydoorCommands() {}

  public static Command open(Baydoor baydoor, Angle angle) {
    return Commands.runEnd(() -> baydoor.setPosition(angle), baydoor::stop, baydoor);
  }
}

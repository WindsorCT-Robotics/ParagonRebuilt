package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
  private final String logKey;
  private final LedIO ledIO;
  private final LedIOInputsAutoLogged ledInputs = new LedIOInputsAutoLogged();
  private Optional<LedPattern> prevPattern = Optional.empty();

  public Led(String name, LedIO ledIO) {
    this.logKey = "Led/" + name;
    this.ledIO = ledIO;
  }

  @Override
  public void periodic() {
    ledIO.updateInputs(ledInputs);
    Logger.processInputs(logKey, ledInputs);
  }

  public void setPattern(LedPattern pattern, Color color, int startIndex, int endIndex) {
    if (prevPattern.isEmpty()) {
      ledIO.setPattern(pattern, color, startIndex, endIndex);
      prevPattern = Optional.of(pattern);
    } else {
      LedPattern presentPrevPattern = prevPattern.get();

      if (!presentPrevPattern.equals(pattern)) {
        ledIO.setPattern(pattern, color, startIndex, endIndex);
      }
      ;
      prevPattern = Optional.of(pattern);
    }
  }

  // ** On setup, a parameter in another subsystem should have LedConsumer. With the LedConsumer,
  // the allocated Led subsystem will supply the setPattern method, thus when the other subsystem
  // accepts the method and pass in a pattern, it will call the setPattern method. */
  @FunctionalInterface
  public static interface LedConsumer {
    public void accept(LedPattern pattern);
  }
}

package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PowerLogger;
import frc.robot.PowerLogger.Device;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Alert disconnectedMotorAlert =
      new Alert("Intake Motor disconnected.", AlertType.kError);

  public Intake(IntakeIO io) {
    this.io = io;
    PowerLogger.addSubsystem("Intake", new Device("Motor", () -> inputs.supplyCurrentAmps));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("~MyInputs/Intake/", inputs);
    disconnectedMotorAlert.set(!inputs.connected);
  }

  public void setVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
    Logger.recordOutput("~MyOutputs/Intake/VelocitySetpoint", velocity);
  }

  public void stop() {
    io.stop();
  }
}

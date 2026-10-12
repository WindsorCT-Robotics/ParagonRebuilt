package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PowerLogger;
import frc.robot.PowerLogger.Device;

public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
    private final Alert motorAlert = new Alert("Kicker Motor Disconnected!", AlertType.kError);

    public Kicker(KickerIO io) {
        this.io = io;
        PowerLogger.addSubsystem("Kicker", new Device("Kicker Motor", () -> inputs.supplyCurrentAmps));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("~MyInputs/Kicker/", inputs);
        motorAlert.set(!inputs.connected);
    }

    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity);
    }

    public void stop() {
        io.stop();
    }
}

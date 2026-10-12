package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PowerLogger;
import frc.robot.PowerLogger.Device;

public class Spindexer extends SubsystemBase {
    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
    private final Alert motorAlert = new Alert("Spindexer Motor Disconnected!", AlertType.kError);
    
    public Spindexer(SpindexerIO io) {
        this.io = io;
        PowerLogger.addSubsystem("Spindexer", new Device("Spindexer Motor", () -> inputs.supplyCurrentAmps));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("~MyInputs/Baydoor/", inputs);
        motorAlert.set(!inputs.connected);
    }

    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity);
    }

    public void stop() {
        io.stop();
    }
}
